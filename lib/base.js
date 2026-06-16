/**
 * BNO08x JavaScript library — transport-agnostic engine.
 *
 * Owns:
 *   - Canonical report id constants and the REPORT_META catalog (single source
 *     of truth for both I2C and IIO transports).
 *   - The readings cache populated by _publish().
 *   - The cached property getters (acceleration, quaternion, ...).
 *   - Generic dispatch via _publish(reportId, data, meta), which emits the
 *     canonical per-report event and a generic 'report' event.
 *
 * Transport-specific code (SHTP packet pump, IIO sysfs handling) lives in
 * lib/sh2.js + lib/i2c.js and lib/iio.js + lib/iio_channels.js.
 */

const { EventEmitter } = require('events');

const sh2 = require('./sh2');

// ---------------------------------------------------------------------
// Report id constants
// ---------------------------------------------------------------------

const BNO_REPORT_ACCELEROMETER = 0x01;
const BNO_REPORT_GYROSCOPE = 0x02;
const BNO_REPORT_MAGNETOMETER = 0x03;
const BNO_REPORT_LINEAR_ACCELERATION = 0x04;
const BNO_REPORT_ROTATION_VECTOR = 0x05;
const BNO_REPORT_GRAVITY = 0x06;
const BNO_REPORT_GAME_ROTATION_VECTOR = 0x08;
const BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09;
const BNO_REPORT_STEP_COUNTER = 0x11;
const BNO_REPORT_STABILITY_CLASSIFIER = 0x13;
const BNO_REPORT_RAW_ACCELEROMETER = 0x14;
const BNO_REPORT_RAW_GYROSCOPE = 0x15;
const BNO_REPORT_RAW_MAGNETOMETER = 0x16;
const BNO_REPORT_SHAKE_DETECTOR = 0x19;
const BNO_REPORT_ACTIVITY_CLASSIFIER = 0x1E;
const BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A;

// ---------------------------------------------------------------------
// Timing / scaling constants
// ---------------------------------------------------------------------

const _DEFAULT_REPORT_INTERVAL = 50000; // microseconds
const _QUAT_READ_TIMEOUT = 0.500;
const _PACKET_READ_TIMEOUT = 2.000;
const _FEATURE_ENABLE_TIMEOUT = 2.0;
const _DEFAULT_TIMEOUT = 2.0;
const _BNO08X_CMD_RESET = 0x01;

const _Q_POINT_14_SCALAR = Math.pow(2, -14);
const _Q_POINT_12_SCALAR = Math.pow(2, -12);
const _Q_POINT_9_SCALAR = Math.pow(2, -9);
const _Q_POINT_8_SCALAR = Math.pow(2, -8);
const _Q_POINT_4_SCALAR = Math.pow(2, -4);

const _ENABLED_ACTIVITIES = 0x1FF;

const REPORT_ACCURACY_STATUS = [
    "Accuracy Unreliable",
    "Low Accuracy",
    "Medium Accuracy",
    "High Accuracy",
];

// ---------------------------------------------------------------------
// REPORT_META — single source of truth for report metadata.
//
// Each entry describes one SH-2 sensor report:
//   key         : short canonical name (also used as the cache property name)
//   eventName   : EventEmitter name used for the per-report event
//   kind        : 'vec3' | 'quat' | 'scalar' | 'classifier' | 'bool'
//   axisCount   : number of words in the SH-2 report payload (3 for vec3,
//                 4 for quat, 1 for scalar/bool/classifier)
//   reportLength: SH-2 report length in bytes (matches CircuitPython table)
//   qShift      : Q-point shift (used by IIO; converted to a multiplicative
//                 scalar for SHTP). 0 means "no scaling" (raw / non-numeric).
//   scalar      : multiplicative scale used by SHTP parser.
//                 = 2^-qShift for scaled reports, 1 for raw/scalar/classifier.
//   isRaw       : true => SHTP parser reads unsigned 16-bit words.
//   accuracy    : true if the SHTP report's status byte carries an accuracy
//                 value that should be passed to listeners.
//   iioBases    : ordered sysfs scan-element base names exposed by the
//                 bno086 kernel driver. null = report is not exposed via IIO.
//   initial     : initial _readings value (deep-copied into the cache when the
//                 report is first registered).
// ---------------------------------------------------------------------

const REPORT_META = Object.freeze({
    [BNO_REPORT_ACCELEROMETER]: {
        key: 'accelerometer',
        eventName: 'accelerometer',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 10,
        qShift: 8,
        scalar: _Q_POINT_8_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_accel_x', 'in_accel_y', 'in_accel_z'],
        initial: [0.0, 0.0, 0.0],
    },
    [BNO_REPORT_GRAVITY]: {
        key: 'gravity',
        eventName: 'gravity',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 10,
        qShift: 8,
        scalar: _Q_POINT_8_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_gravity_x', 'in_gravity_y', 'in_gravity_z'],
        initial: [0.0, 0.0, 0.0],
    },
    [BNO_REPORT_GYROSCOPE]: {
        key: 'gyroscope',
        eventName: 'gyroscope',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 10,
        qShift: 9,
        scalar: _Q_POINT_9_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_anglvel_x', 'in_anglvel_y', 'in_anglvel_z'],
        initial: [0.0, 0.0, 0.0],
    },
    [BNO_REPORT_MAGNETOMETER]: {
        key: 'magnetometer',
        eventName: 'magnetometer',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 10,
        qShift: 4,
        scalar: _Q_POINT_4_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_magn_x', 'in_magn_y', 'in_magn_z'],
        initial: [0.0, 0.0, 0.0],
    },
    [BNO_REPORT_LINEAR_ACCELERATION]: {
        key: 'linear_acceleration',
        eventName: 'linear_acceleration',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 10,
        qShift: 8,
        scalar: _Q_POINT_8_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_accel1_x', 'in_accel1_y', 'in_accel1_z'],
        initial: [0.0, 0.0, 0.0],
    },
    [BNO_REPORT_ROTATION_VECTOR]: {
        key: 'quaternion',
        eventName: 'quaternion',
        kind: 'quat',
        axisCount: 4,
        reportLength: 14,
        qShift: 14,
        scalar: _Q_POINT_14_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_rot1_quaternion'],
        initial: [0.0, 0.0, 0.0, 0.0],
    },
    [BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR]: {
        key: 'geomagnetic_quaternion',
        eventName: 'geomagnetic_quaternion',
        kind: 'quat',
        axisCount: 4,
        reportLength: 14,
        qShift: 12,
        scalar: _Q_POINT_12_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_rot2_quaternion'],
        initial: [0.0, 0.0, 0.0, 0.0],
    },
    [BNO_REPORT_GAME_ROTATION_VECTOR]: {
        key: 'game_quaternion',
        eventName: 'game_quaternion',
        kind: 'quat',
        axisCount: 4,
        reportLength: 12,
        qShift: 14,
        scalar: _Q_POINT_14_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: ['in_rot0_quaternion'],
        initial: [0.0, 0.0, 0.0, 0.0],
    },
    [BNO_REPORT_STEP_COUNTER]: {
        key: 'step_counter',
        eventName: 'step_counter',
        kind: 'scalar',
        axisCount: 1,
        reportLength: 12,
        qShift: 0,
        scalar: 1,
        isRaw: false,
        accuracy: false,
        iioBases: null,
        initial: 0,
    },
    [BNO_REPORT_SHAKE_DETECTOR]: {
        key: 'shake',
        eventName: 'shake',
        kind: 'bool',
        axisCount: 1,
        reportLength: 6,
        qShift: 0,
        scalar: 1,
        isRaw: false,
        accuracy: false,
        iioBases: null,
        initial: false,
    },
    [BNO_REPORT_STABILITY_CLASSIFIER]: {
        key: 'stability_classification',
        eventName: 'stability_classification',
        kind: 'classifier',
        axisCount: 1,
        reportLength: 6,
        qShift: 0,
        scalar: 1,
        isRaw: false,
        accuracy: false,
        iioBases: null,
        initial: 'Unknown',
    },
    [BNO_REPORT_ACTIVITY_CLASSIFIER]: {
        key: 'activity_classification',
        eventName: 'activity_classification',
        kind: 'classifier',
        axisCount: 1,
        reportLength: 16,
        qShift: 0,
        scalar: 1,
        isRaw: false,
        accuracy: false,
        iioBases: null,
        initial: {
            "Tilting": -1,
            "most_likely": "Unknown",
            "OnStairs": -1,
            "On-Foot": -1,
            "Other": -1,
            "On-Bicycle": -1,
            "Still": -1,
            "Walking": -1,
            "Unknown": -1,
            "Running": -1,
            "In-Vehicle": -1,
        },
    },
    [BNO_REPORT_RAW_ACCELEROMETER]: {
        key: 'raw_accelerometer',
        eventName: 'raw_accelerometer',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 16,
        qShift: 0,
        scalar: 1,
        isRaw: true,
        accuracy: true,
        iioBases: ['in_accel2_x', 'in_accel2_y', 'in_accel2_z'],
        initial: [0, 0, 0],
    },
    [BNO_REPORT_RAW_GYROSCOPE]: {
        key: 'raw_gyroscope',
        eventName: 'raw_gyroscope',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 16,
        qShift: 0,
        scalar: 1,
        isRaw: true,
        accuracy: true,
        iioBases: ['in_anglvel1_x', 'in_anglvel1_y', 'in_anglvel1_z'],
        initial: [0, 0, 0],
    },
    [BNO_REPORT_RAW_MAGNETOMETER]: {
        key: 'raw_magnetometer',
        eventName: 'raw_magnetometer',
        kind: 'vec3',
        axisCount: 3,
        reportLength: 16,
        qShift: 0,
        scalar: 1,
        isRaw: true,
        accuracy: true,
        iioBases: ['in_magn1_x', 'in_magn1_y', 'in_magn1_z'],
        initial: [0, 0, 0],
    },
    [BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR]: {
        key: 'gyro_integrated_rotation_vector',
        eventName: 'gyro_integrated_rotation_vector',
        kind: 'quat',
        axisCount: 4,
        reportLength: 14,
        qShift: 14,
        scalar: _Q_POINT_14_SCALAR,
        isRaw: false,
        accuracy: true,
        iioBases: null,
        initial: [0.0, 0.0, 0.0, 0.0],
    },
});

// Numeric report id -> meta.
const REPORT_META_BY_ID = REPORT_META;

// Convenience derived tables.
const _RAW_REPORTS = Object.freeze({
    [BNO_REPORT_RAW_ACCELEROMETER]: BNO_REPORT_ACCELEROMETER,
    [BNO_REPORT_RAW_GYROSCOPE]: BNO_REPORT_GYROSCOPE,
    [BNO_REPORT_RAW_MAGNETOMETER]: BNO_REPORT_MAGNETOMETER,
});

function getReportMeta(reportId) {
    return REPORT_META_BY_ID[reportId] || null;
}

function getSensorReportLength(reportId) {
    const meta = REPORT_META_BY_ID[reportId];
    return meta ? meta.reportLength : null;
}

// ---------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------

class BNO08X extends EventEmitter {
    constructor({ debug = false } = {}) {
        super();
        this._debug = Boolean(debug);
        this._readings = {};
    }

    /**
     * Update the readings cache and emit:
     *   - the canonical per-report event with the I2C-compatible signature
     *     (data, accuracy) so existing I2C listeners keep working.
     *   - a generic 'report' event with a structured payload that includes
     *     both the engine `key` and any transport-supplied extras (timestamp,
     *     raw words, qShift, ...).
     *
     * @param {number} reportId   - SHTP report id (BNO_REPORT_*)
     * @param {*}      data       - decoded value (array, number, string, ...)
     * @param {Object} [meta]
     * @param {number|null} [meta.accuracy=null]
     * @param {Object}      [meta.extra]   transport-specific payload extras
     */
    _publish(reportId, data, meta = {}) {
        const reportMeta = REPORT_META_BY_ID[reportId];
        if (!reportMeta) {
            // Unknown id — emit a fallthrough event so nothing is silently lost.
            this.emit('sensor_data', { reportId, data, accuracy: meta.accuracy ?? null });
            this.emit('report', {
                type: 'unknown',
                reportKey: null,
                reportId,
                data,
                accuracy: meta.accuracy ?? null,
                ...meta.extra,
            });
            return;
        }

        this._readings[reportId] = data;

        const { accuracy = null, extra = null } = meta;

        if (reportMeta.accuracy) {
            this.emit(reportMeta.eventName, data, accuracy);
        } else {
            this.emit(reportMeta.eventName, data);
        }

        this.emit('report', {
            type: reportMeta.eventName,
            reportKey: reportMeta.key,
            reportId,
            kind: reportMeta.kind,
            data,
            accuracy,
            ...(extra || {}),
        });
    }

    /**
     * Hook overridden by the I2C transport so that `imu.acceleration` (and
     * friends) trigger a packet pump. IIO populates the cache asynchronously
     * via its own poll loop, so this is a no-op there.
     */
    _refreshCache() {}

    _readCache(reportId, label) {
        this._refreshCache();
        if (!(reportId in this._readings)) {
            throw new Error(`No ${label} report found, is it enabled?`);
        }
        return this._readings[reportId];
    }

    // -----------------------------------------------------------------
    // Cached property getters (work on both transports).
    // -----------------------------------------------------------------

    get acceleration() {
        return this._readCache(BNO_REPORT_ACCELEROMETER, 'accel');
    }

    get gravity() {
        return this._readCache(BNO_REPORT_GRAVITY, 'gravity');
    }

    get gyro() {
        return this._readCache(BNO_REPORT_GYROSCOPE, 'gyro');
    }

    get magnetic() {
        return this._readCache(BNO_REPORT_MAGNETOMETER, 'magfield');
    }

    get linear_acceleration() {
        return this._readCache(BNO_REPORT_LINEAR_ACCELERATION, 'lin. accel');
    }

    get quaternion() {
        return this._readCache(BNO_REPORT_ROTATION_VECTOR, 'quaternion');
    }

    get geomagnetic_quaternion() {
        return this._readCache(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR, 'geomag quaternion');
    }

    get game_quaternion() {
        return this._readCache(BNO_REPORT_GAME_ROTATION_VECTOR, 'game quaternion');
    }

    get steps() {
        return this._readCache(BNO_REPORT_STEP_COUNTER, 'steps');
    }

    get raw_acceleration() {
        return this._readCache(BNO_REPORT_RAW_ACCELEROMETER, 'raw acceleration');
    }

    get raw_gyro() {
        return this._readCache(BNO_REPORT_RAW_GYROSCOPE, 'raw gyro');
    }

    get raw_magnetic() {
        return this._readCache(BNO_REPORT_RAW_MAGNETOMETER, 'raw magnetic');
    }

    get stability_classification() {
        return this._readCache(BNO_REPORT_STABILITY_CLASSIFIER, 'stability classification');
    }

    get activity_classification() {
        return this._readCache(BNO_REPORT_ACTIVITY_CLASSIFIER, 'activity classification');
    }

    get shake() {
        this._refreshCache();
        if (!(BNO_REPORT_SHAKE_DETECTOR in this._readings)) {
            throw new Error('No shake report found, is it enabled?');
        }
        const detected = this._readings[BNO_REPORT_SHAKE_DETECTOR];
        if (detected) {
            this._readings[BNO_REPORT_SHAKE_DETECTOR] = false;
        }
        return detected;
    }

    // -----------------------------------------------------------------
    // Misc helpers shared by both transports.
    // -----------------------------------------------------------------

    _dbg(...args) {
        if (this._debug) {
            console.log('DBG::\t\t', ...args);
        }
    }

    _sleep(seconds) {
        return new Promise((resolve) => setTimeout(resolve, seconds * 1000));
    }

    _sleep_sync(seconds) {
        const start = Date.now();
        while (Date.now() - start < seconds * 1000) {
            // busy wait
        }
    }
}

function _elapsed(start_time) {
    return Date.now() / 1000.0 - start_time;
}

module.exports = {
    // Engine
    BNO08X,

    // Report id constants
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_STABILITY_CLASSIFIER,
    BNO_REPORT_RAW_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE,
    BNO_REPORT_RAW_MAGNETOMETER,
    BNO_REPORT_SHAKE_DETECTOR,
    BNO_REPORT_ACTIVITY_CLASSIFIER,
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR,

    // Catalog / accessors
    REPORT_META,
    REPORT_META_BY_ID,
    getReportMeta,
    getSensorReportLength,
    _RAW_REPORTS,

    // Status / timing
    REPORT_ACCURACY_STATUS,
    _DEFAULT_REPORT_INTERVAL,
    _QUAT_READ_TIMEOUT,
    _PACKET_READ_TIMEOUT,
    _FEATURE_ENABLE_TIMEOUT,
    _DEFAULT_TIMEOUT,
    _BNO08X_CMD_RESET,
    _ENABLED_ACTIVITIES,
    _elapsed,

    // Re-export SHTP symbols (back-compat for any existing imports of base.js)
    ...sh2,

    // Re-export parsers under their old names so external code that imported
    // helpers from base.js keeps working.
    parse_sensor_id: sh2.parse_sensor_id,
    _parse_sensor_report_data: sh2._parse_sensor_report_data,
    _parse_step_counter_report: sh2._parse_step_counter_report,
    _parse_stability_classifier_report: sh2._parse_stability_classifier_report,
    _parse_activity_classifier_report: sh2._parse_activity_classifier_report,
    _parse_shake_report: sh2._parse_shake_report,
};
