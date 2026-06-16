/**
 * BNO08x Node.js Library
 *
 * Port of the adafruit_bno08x Python library for the Hillcrest Laboratories
 * BNO08x family of IMUs.
 *
 * @author Sean Sullivan
 */

const { BNO08X_I2C, _BNO08X_DEFAULT_ADDRESS } = require('./lib/i2c');
const { BNO08X_IIO } = require('./lib/iio');
const {
    BNO08X,
    Packet,
    PacketError,
    PacketHeader,

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

    REPORT_ACCURACY_STATUS,
    REPORT_META,

    parse_sensor_id,
    _parse_sensor_report_data,
    _parse_step_counter_report,
    _parse_stability_classifier_report,
    _parse_activity_classifier_report,
    _parse_shake_report,
} = require('./lib/base');

module.exports = {
    // Main classes
    BNO08X,
    BNO08X_I2C,
    BNO08X_IIO,

    // Supporting classes
    Packet,
    PacketError,
    PacketHeader,

    // Constants
    BNO08X_DEFAULT_ADDRESS: _BNO08X_DEFAULT_ADDRESS,

    // Report ids (use these for both transports' enable_feature)
    REPORTS: {
        ACCELEROMETER: BNO_REPORT_ACCELEROMETER,
        GYROSCOPE: BNO_REPORT_GYROSCOPE,
        MAGNETOMETER: BNO_REPORT_MAGNETOMETER,
        LINEAR_ACCELERATION: BNO_REPORT_LINEAR_ACCELERATION,
        ROTATION_VECTOR: BNO_REPORT_ROTATION_VECTOR,
        GRAVITY: BNO_REPORT_GRAVITY,
        GAME_ROTATION_VECTOR: BNO_REPORT_GAME_ROTATION_VECTOR,
        GEOMAGNETIC_ROTATION_VECTOR: BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
        STEP_COUNTER: BNO_REPORT_STEP_COUNTER,
        STABILITY_CLASSIFIER: BNO_REPORT_STABILITY_CLASSIFIER,
        RAW_ACCELEROMETER: BNO_REPORT_RAW_ACCELEROMETER,
        RAW_GYROSCOPE: BNO_REPORT_RAW_GYROSCOPE,
        RAW_MAGNETOMETER: BNO_REPORT_RAW_MAGNETOMETER,
        SHAKE_DETECTOR: BNO_REPORT_SHAKE_DETECTOR,
        ACTIVITY_CLASSIFIER: BNO_REPORT_ACTIVITY_CLASSIFIER,
        GYRO_INTEGRATED_ROTATION_VECTOR: BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR,
    },

    // Accuracy status string table
    ACCURACY_STATUS: REPORT_ACCURACY_STATUS,

    // Canonical report metadata table (single source of truth)
    REPORT_META,

    // Helper functions
    parse_sensor_id,
    parse_sensor_report_data: _parse_sensor_report_data,
    parse_step_counter_report: _parse_step_counter_report,
    parse_stability_classifier_report: _parse_stability_classifier_report,
    parse_activity_classifier_report: _parse_activity_classifier_report,
    parse_shake_report: _parse_shake_report,
};
