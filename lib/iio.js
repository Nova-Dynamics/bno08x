/**
 * BNO08x Linux IIO transport.
 *
 * Reads the SH-2 reports exposed by the bno086 kernel driver through the IIO
 * subsystem and forwards them to the shared engine. Public API mirrors the
 * I2C transport (`enable_feature(reportId, interval_us)`, `disable_feature`,
 * cached property getters).
 *
 * Channel discovery and the byte-layout parse plan are computed from sysfs
 * scan_elements; record decoding handles the fixed per-record buffer layout
 * documented in /memories/repo/node-handoff-multi-report.md.
 */

const fs = require('fs');
const path = require('path');

const {
    BNO08X,
    REPORT_META_BY_ID,
    _DEFAULT_REPORT_INTERVAL,
} = require('./base');

const {
    TIMESTAMP_BASE,
    IIO_REPORT_BY_ID,
    lookupSysfsBase,
    getIioReport,
} = require('./iio_channels');

const MIN_RECORD_SIZE = 8;
const MAX_RECORD_SIZE = 256;
const READ_BATCH_RECORDS = 16;
const SCAN_TYPE_RE = /^([bl]e):([suf])(\d+)\/(\d+)(?:X(\d+))?>>\d+$/;

class BNO08X_IIO extends BNO08X {
    /**
     * @param {number|string} device  - iio device index, "iio:deviceX",
     *                                  or "/dev/iio:deviceX".
     * @param {Object} [options]
     * @param {string}  [options.sysfsRoot='/sys/bus/iio/devices']
     * @param {number}  [options.pollIntervalMs=20]
     * @param {number|null} [options.bufferLength=null]
     * @param {boolean} [options.debug=false]
     * @param {boolean} [options.debugRaw=false]
     * @param {number}  [options.debugRawMaxRecords=200]
     * @param {number}  [options.quaternionNormTolerance=0.25]
     */
    constructor(device, options = {}) {
        const {
            sysfsRoot = '/sys/bus/iio/devices',
            pollIntervalMs = 20,
            bufferLength = null,
            debug = false,
            debugRaw = false,
            debugRawMaxRecords = 200,
            quaternionNormTolerance = 0.25,
        } = options;

        super({ debug });

        this._debugRaw = Boolean(debugRaw);
        this._debugRawMaxRecords = Number(debugRawMaxRecords);
        this._quaternionNormTolerance = Number(quaternionNormTolerance);

        this._pollIntervalMs = pollIntervalMs;
        this._bufferLength = bufferLength;

        this._sysfsRoot = sysfsRoot;
        this._deviceNode = this._normalizeDeviceNode(device);
        this._sysfsDeviceName = path.basename(this._deviceNode);
        this._sysfsDevicePath = path.join(this._sysfsRoot, this._sysfsDeviceName);

        this._fd = null;
        this._pollTimer = null;
        this._pending = Buffer.alloc(0);
        this._recordsSeen = 0;

        // reportId -> { meta, axes: [{sysfsBase, axisIndex, scanIndex, type, ...}] }
        this._reports = new Map();
        this._timestamp = null;
        this._bufferEnablePath = null;
        this._bufferLengthPath = null;
        this._scanBytesPath = null;

        this._recordSize = 0;
        this._parsePlan = [];
        this._buffered = false;

        this._discoverDevice();
        this._openDevice();
    }

    // -----------------------------------------------------------------
    // Discovery
    // -----------------------------------------------------------------

    _normalizeDeviceNode(device) {
        if (Number.isInteger(device) && device >= 0) {
            return `/dev/iio:device${device}`;
        }
        if (typeof device !== 'string' || device.length === 0) {
            throw new Error('device must be a non-negative index, "iio:deviceX", or "/dev/iio:deviceX"');
        }
        if (device.startsWith('/dev/iio:device')) return device;
        if (device.startsWith('iio:device')) return path.join('/dev', device);
        throw new Error(`Unsupported IIO device identifier: ${device}`);
    }

    _openDevice() {
        try {
            this._fd = fs.openSync(this._deviceNode, fs.constants.O_RDONLY | fs.constants.O_NONBLOCK);
            this._dbg(`opened ${this._deviceNode}`);
        } catch (error) {
            throw new Error(`Failed to open IIO device ${this._deviceNode}: ${error.message}`);
        }
    }

    _resolveFirstExisting(candidates) {
        for (const rel of candidates) {
            if (fs.existsSync(path.join(this._sysfsDevicePath, rel))) return rel;
        }
        return null;
    }

    _readSysfs(rel) {
        return fs.readFileSync(path.join(this._sysfsDevicePath, rel), 'utf8').trim();
    }

    _writeSysfs(rel, value) {
        fs.writeFileSync(path.join(this._sysfsDevicePath, rel), String(value));
    }

    _parseScanType(typeRaw) {
        const m = SCAN_TYPE_RE.exec(typeRaw);
        if (!m) return null;
        const storageBits = Number.parseInt(m[4], 10);
        if (!Number.isInteger(storageBits) || storageBits <= 0 || storageBits % 8 !== 0) return null;
        const repeat = m[5] ? Number.parseInt(m[5], 10) : 1;
        const storageBytes = storageBits / 8;
        return {
            raw: typeRaw,
            endianness: m[1],
            sign: m[2],
            realBits: Number.parseInt(m[3], 10),
            storageBits,
            storageBytes,
            repeat,
            totalBytes: storageBytes * repeat,
        };
    }

    _discoverDevice() {
        if (!fs.existsSync(this._sysfsDevicePath)) {
            throw new Error(`IIO sysfs path not found: ${this._sysfsDevicePath}`);
        }

        this._bufferEnablePath = this._resolveFirstExisting(['buffer/enable', 'buffer0/enable']);
        if (!this._bufferEnablePath) {
            throw new Error(`buffer enable attribute not found under ${this._sysfsDevicePath}`);
        }
        this._bufferLengthPath = this._resolveFirstExisting(['buffer/length', 'buffer0/length']);
        this._scanBytesPath = this._resolveFirstExisting(['buffer/scan_bytes', 'buffer0/scan_bytes', 'scan_bytes']);

        const scanDir = path.join(this._sysfsDevicePath, 'scan_elements');
        if (!fs.existsSync(scanDir)) {
            throw new Error(`scan_elements directory not found under ${this._sysfsDevicePath}`);
        }

        const indexFiles = fs.readdirSync(scanDir).filter((n) => n.endsWith('_index'));

        // Group raw axis entries into reports based on the catalog.
        const reportInProgress = new Map(); // reportId -> { entry, axes: Map<axisIdx, axisInfo> }

        for (const indexFile of indexFiles) {
            const base = indexFile.slice(0, -'_index'.length);

            const indexPath = `scan_elements/${indexFile}`;
            const typePath = `scan_elements/${base}_type`;
            const enPath = `scan_elements/${base}_en`;

            const scanIndex = Number.parseInt(this._readSysfs(indexPath), 10);
            const typeRaw = this._readSysfs(typePath);
            const type = this._parseScanType(typeRaw);
            if (!Number.isInteger(scanIndex) || !type) {
                this._dbg(`skip ${base}: bad index=${scanIndex} type=${typeRaw}`);
                continue;
            }

            if (base === TIMESTAMP_BASE) {
                this._timestamp = { sysfsBase: base, scanIndex, type, indexPath, typePath, enPath };
                continue;
            }

            const lookup = lookupSysfsBase(base);
            if (!lookup) {
                this._dbg(`skip ${base}: not in catalog`);
                continue;
            }
            const { reportId, meta, axisIndex } = lookup;

            let inProg = reportInProgress.get(reportId);
            if (!inProg) {
                inProg = { reportId, meta, axes: new Map() };
                reportInProgress.set(reportId, inProg);
            }
            inProg.axes.set(axisIndex, {
                sysfsBase: base,
                axisIndex,
                scanIndex,
                type,
                indexPath,
                typePath,
                enPath,
                samplingFreqPath: `${base}_sampling_frequency`,
                scalePath: `${base}_scale`,
            });
        }

        if (!this._timestamp) {
            throw new Error('IIO timestamp scan element not found (in_timestamp_index missing)');
        }

        for (const [reportId, { meta, axes }] of reportInProgress) {
            if (axes.size !== meta.iioBases.length) {
                this._dbg(`drop reportId=0x${reportId.toString(16)}: have ${axes.size}/${meta.iioBases.length} axes`);
                continue;
            }
            const ordered = [];
            let ok = true;
            for (let i = 0; i < meta.iioBases.length; i += 1) {
                const a = axes.get(i);
                if (!a) {
                    this._dbg(`drop reportId=0x${reportId.toString(16)}: axis ${i} (${meta.iioBases[i]}) missing`);
                    ok = false;
                    break;
                }
                ordered.push(a);
            }
            if (!ok) continue;

            // Sanity: quat reports should have a single repeat=4 element;
            // vec3 reports should have three repeat=1 elements.
            if (meta.kind === 'quat' && ordered[0].type.repeat !== 4) {
                this._dbg(`drop reportId=0x${reportId.toString(16)}: quat repeat=${ordered[0].type.repeat} expected 4`);
                continue;
            }
            if (meta.kind === 'vec3' && ordered.some((a) => a.type.repeat !== 1)) {
                this._dbg(`drop reportId=0x${reportId.toString(16)}: vec3 axis repeat != 1`);
                continue;
            }

            this._reports.set(reportId, {
                meta,
                axes: ordered,
                samplingFreqPath: ordered[0].samplingFreqPath,
                scalePath: ordered[0].scalePath,
            });
        }

        this._dbg(`discovered reports: ${[...this._reports.keys()].map((id) => `0x${id.toString(16)}`).join(', ') || '(none)'}`);
    }

    // -----------------------------------------------------------------
    // Public API (mirrors I2C)
    // -----------------------------------------------------------------

    /**
     * Enable an SH-2 report. Mirrors `BNO08X_I2C.enable_feature`.
     *
     * @param {number} feature_id          BNO_REPORT_* numeric id (use REPORTS.X).
     * @param {number} [report_interval]   sampling period in microseconds.
     */
    async enable_feature(feature_id, report_interval = _DEFAULT_REPORT_INTERVAL) {
        const info = this._requireReport(feature_id);
        const wasRunning = this._buffered;

        if (wasRunning) {
            this._stopBuffer();
        }

        for (const axis of info.axes) {
            this._writeSysfs(axis.enPath, 1);
        }

        if (Number.isFinite(report_interval) && report_interval > 0) {
            const hz = Math.max(1, Math.round(1e6 / report_interval));
            try {
                this._writeSysfs(info.samplingFreqPath, hz);
            } catch (err) {
                this._dbg(`sampling_frequency write for 0x${feature_id.toString(16)} failed: ${err.message}`);
            }
        }

        // Seed the readings cache so subsequent getter access does not throw
        // before the first record arrives.
        if (!(feature_id in this._readings)) {
            this._readings[feature_id] = deepCopy(info.meta.initial);
        }

        this._startBuffer();
    }

    /**
     * Disable an SH-2 report. The buffer/poll loop is stopped automatically
     * when the last enabled report is removed.
     *
     * @param {number} feature_id  BNO_REPORT_* numeric id.
     */
    async disable_feature(feature_id) {
        const info = this._requireReport(feature_id);
        if (this._buffered) {
            this._stopBuffer();
        }
        for (const axis of info.axes) {
            this._writeSysfs(axis.enPath, 0);
        }
        delete this._readings[feature_id];
        if (this.getEnabledReports().length > 0) {
            this._startBuffer();
        }
    }

    // -----------------------------------------------------------------
    // Calibration (parity with BNO08X_I2C)
    // -----------------------------------------------------------------

    /**
     * Enable on-device dynamic calibration for accel, gyro, and mag.
     * Mirrors `BNO08X_I2C.begin_calibration()`. The kernel exposes the
     * underlying SH-2 cal-config bitmask via `calibration_auto`; this
     * helper writes the accel|gyro|mag bits (0x07).
     */
    begin_calibration() {
        this.setCalibrationAutoMask(BNO08X_IIO.CAL_ACCEL | BNO08X_IIO.CAL_GYRO | BNO08X_IIO.CAL_MAG);
    }

    /**
     * Magnetometer calibration accuracy (0..3). Mirrors the I2C
     * `calibration_status` getter, which returns the magnetometer
     * accuracy field. Read directly from the per-channel sysfs hook the
     * kernel publishes for each calibrated axis.
     */
    get calibration_status() {
        return this._readCalibrationAccuracy('in_magn_x');
    }

    /**
     * Persist the current Dynamic Calibration Data to the device's
     * non-volatile flash. Mirrors `BNO08X_I2C.save_calibration_data()`.
     * Resolves once the kernel reports the underlying SH-2 op succeeded.
     *
     * (Kept async for I2C-API parity; the sysfs write is synchronous.)
     */
    async save_calibration_data() {
        try {
            this._writeSysfs('calibration_save', 1);
        } catch (err) {
            throw new Error(`Could not save calibration data: ${err.message}`);
        }
    }

    // ----- IIO-only extensions (no I2C equivalent) -----

    /**
     * Read the device-wide auto-calibration sensor mask. Returns the raw
     * SH2_CAL_* bitmask (ACCEL=0x01, GYRO=0x02, MAG=0x04, PLANAR=0x08,
     * ON_TABLE=0x10).
     */
    getCalibrationAutoMask() {
        const raw = this._readSysfs('calibration_auto');
        const v = Number.parseInt(raw, raw.startsWith('0x') ? 16 : 10);
        if (!Number.isInteger(v)) {
            throw new Error(`unexpected calibration_auto value: ${raw}`);
        }
        return v;
    }

    /**
     * Write the device-wide auto-calibration sensor mask. Use the
     * `BNO08X_IIO.CAL_*` constants (ACCEL/GYRO/MAG/PLANAR/ON_TABLE).
     */
    setCalibrationAutoMask(mask) {
        if (!Number.isInteger(mask) || mask < 0 || mask > 0xff) {
            throw new Error('mask must be a u8');
        }
        this._writeSysfs('calibration_auto', `0x${mask.toString(16)}`);
    }

    /**
     * Per-sensor calibration accuracy snapshot (0..3). Reads the
     * per-channel `*_calibration_accuracy` ext_info that the kernel
     * exposes on the calibrated accel, gyro, and mag axes.
     */
    getCalibrationAccuracies() {
        return {
            accelerometer: this._readCalibrationAccuracy('in_accel_x'),
            gyroscope: this._readCalibrationAccuracy('in_anglvel_x'),
            magnetometer: this._readCalibrationAccuracy('in_magn_x'),
        };
    }

    /**
     * Clear DCD in RAM and reset the sensor hub. The kernel re-arms any
     * reports that were streaming before the reset; userspace should
     * still expect a brief gap in the data stream.
     */
    async clear_calibration_data() {
        try {
            this._writeSysfs('calibration_reset', 1);
        } catch (err) {
            throw new Error(`Could not clear calibration data: ${err.message}`);
        }
    }

    /**
     * Apply a sensor reorientation (tare) by writing the kernel's
     * `reorientation_quaternion` sysfs attribute.
     *
     * @param {{x:number, y:number, z:number, w:number}} q Unit quaternion.
     *   Pass {x:0,y:0,z:0,w:1} (identity) to clear the tare.
     *
     * The kernel takes Q14 signed 16-bit integers (the SH-2 wire format)
     * and never touches floating point itself, so this helper does the
     * float -> Q14 conversion: clamp(round(v * 16384), -32768, 32767).
     */
    setReorientationQuaternion(q) {
        if (!q || typeof q !== 'object') {
            throw new TypeError('setReorientationQuaternion: expected {x,y,z,w}');
        }
        const components = ['x', 'y', 'z', 'w'].map((key) => {
            const v = q[key];
            if (typeof v !== 'number' || !Number.isFinite(v)) {
                throw new TypeError(
                    `setReorientationQuaternion: ${key} must be a finite number`);
            }
            const scaled = Math.round(v * 16384);
            return Math.max(-32768, Math.min(32767, scaled));
        });
        try {
            this._writeSysfs('reorientation_quaternion', components.join(' '));
        } catch (err) {
            throw new Error(
                `Could not set reorientation quaternion: ${err.message}`);
        }
    }

    /**
     * Convenience: tare the sensor so the current direction of gravity
     * becomes +Z (Z-up convention). Reads the gravity report stream,
     * averages a few samples, computes the shortest-arc rotation that
     * maps measured gravity onto +Z, and applies it via
     * `setReorientationQuaternion`.
     *
     * NOTE on the X/Y swap: empirically (see `test_reorient_sweep.js`)
     * the BNO086 firmware on this part interprets the SET_REORIENTATION
     * quaternion in an axis frame whose X and Y are exchanged relative
     * to the gravity-report frame. The five `frs_*_orientation` records
     * are unprogrammed (firmware default = identity), so this is a
     * firmware ABI quirk, not an FRS misconfig. We compensate here by
     * swapping qx and qy on the wire.
     *
     * @param {object} [opts]
     * @param {number} [opts.samples=20]   Gravity samples to average.
     * @param {number} [opts.timeoutMs=2000] Max wait for the samples.
     * @param {number} [opts.intervalUs=20000] Report interval to request
     *   if gravity isn't already streaming. Ignored if gravity is enabled.
     * @returns {Promise<{quaternion:{x:number,y:number,z:number,w:number},
     *                    gravityBefore:number[]}>}
     */
    async tareGravityToZ(opts = {}) {
        const samples    = Number.isInteger(opts.samples)   ? opts.samples   : 20;
        const timeoutMs  = Number.isInteger(opts.timeoutMs) ? opts.timeoutMs : 2000;
        const intervalUs = Number.isInteger(opts.intervalUs) ? opts.intervalUs : 20000;

        const BNO_REPORT_GRAVITY = 0x06;
        const wasEnabled = this.getEnabledReports().includes(BNO_REPORT_GRAVITY);
        if (!wasEnabled) {
            await this.enable_feature(BNO_REPORT_GRAVITY, intervalUs);
        }

        let g;
        try {
            g = await new Promise((resolve, reject) => {
                const acc = [0, 0, 0];
                let got = 0;
                const timer = setTimeout(() => {
                    cleanup();
                    reject(new Error(`tareGravityToZ: only got ${got}/${samples} gravity samples in ${timeoutMs} ms`));
                }, timeoutMs);
                const cleanup = () => {
                    clearTimeout(timer);
                    this.off('report', onReport);
                    this.off('error',  onError);
                };
                const onReport = (report) => {
                    if (report.reportId !== BNO_REPORT_GRAVITY || report.kind !== 'vec3') return;
                    acc[0] += report.data[0];
                    acc[1] += report.data[1];
                    acc[2] += report.data[2];
                    got++;
                    if (got >= samples) {
                        cleanup();
                        resolve([acc[0]/got, acc[1]/got, acc[2]/got]);
                    }
                };
                const onError = (err) => { cleanup(); reject(err); };
                this.on('report', onReport);
                this.on('error',  onError);
            });
        } finally {
            if (!wasEnabled) {
                try { await this.disable_feature(BNO_REPORT_GRAVITY); } catch (_) { /* ignore */ }
            }
        }

        const gn = Math.hypot(g[0], g[1], g[2]);
        if (!(gn > 0.5)) {
            throw new Error(`tareGravityToZ: gravity magnitude ${gn.toFixed(3)} is too small to tare`);
        }
        const u = [g[0]/gn, g[1]/gn, g[2]/gn];

        // Shortest-arc quaternion that rotates +Z onto u.
        // r = (1 + Z.u, Z x u) normalised; with Z = [0,0,1]:
        //   Z.u = u[2]; Z x u = [-u[1], u[0], 0].
        const rw = 1 + u[2];
        let rx = -u[1], ry = u[0], rz = 0;
        if (rw < 1e-6) {
            // Antipodal: gravity points along -Z; pick X as the rotation axis.
            rx = 1; ry = 0; rz = 0;
        }
        const rn = Math.hypot(rx, ry, rz, rw);
        const r = { x: rx/rn, y: ry/rn, z: rz/rn, w: rw/rn };

        // Apply firmware-quirk X/Y swap.
        const q = { x: r.y, y: r.x, z: r.z, w: r.w };
        this.setReorientationQuaternion(q);
        return { quaternion: q, gravityBefore: g };
    }

    /**
     * Read one of the BNO086 axis-orientation FRS records.
     *
     *   imu.getOrientationFrs('system') -> { name, sysfs, words, x, y, z, w, isIdentity, source }
     *
     * `name` is one of: 'system', 'accel', 'gyro', 'magn', 'screen_accel'.
     *
     * If the kernel returns an empty body (zero bytes) the FRS record is
     * unprogrammed and the firmware uses its built-in identity default;
     * we surface that as `{ source: 'default', isIdentity: true, words: [],
     * x: 0, y: 0, z: 0, w: 1 }`. A populated record is decoded from Q30
     * (x, y, z, w) and surfaced as `{ source: 'frs' }`.
     */
    getOrientationFrs(name) {
        const map = {
            system:       'frs_system_orientation',
            accel:        'frs_accel_orientation',
            gyro:         'frs_gyro_orientation',
            magn:         'frs_mag_orientation',
            screen_accel: 'frs_screen_accel_orientation',
        };
        const rel = map[name];
        if (!rel) {
            throw new Error(
                `getOrientationFrs: unknown record "${name}"; expected one of ${Object.keys(map).join(', ')}`);
        }
        const raw = this._readSysfs(rel);
        if (raw.length === 0) {
            return {
                name, sysfs: rel, source: 'default',
                words: [], x: 0, y: 0, z: 0, w: 1,
                isIdentity: true,
            };
        }
        const parts = raw.split(/\s+/).filter(Boolean);
        if (parts.length < 4) {
            throw new Error(`getOrientationFrs(${name}): unexpected sysfs output: ${raw}`);
        }
        const words = parts.slice(0, 4).map((s) => {
            const v = Number.parseInt(s, 16);
            if (!Number.isFinite(v)) {
                throw new Error(`getOrientationFrs(${name}): bad word "${s}"`);
            }
            return v >>> 0; // force u32
        });
        const Q30 = 1 / (1 << 30);
        const decode = (u) => {
            const s = (u | 0); // sign-extend back to s32
            return s * Q30;
        };
        const quat = {
            x: decode(words[0]),
            y: decode(words[1]),
            z: decode(words[2]),
            w: decode(words[3]),
        };
        const tol = 1e-6;
        const isIdentity = Math.abs(quat.x) < tol &&
                           Math.abs(quat.y) < tol &&
                           Math.abs(quat.z) < tol &&
                           Math.abs(quat.w - 1) < tol;
        return {
            name, sysfs: rel, source: 'frs',
            words, ...quat, isIdentity,
        };
    }

    _readCalibrationAccuracy(sysfsBase) {
        const raw = this._readSysfs(`${sysfsBase}_calibration_accuracy`);
        const v = Number.parseInt(raw, 10);
        if (!Number.isInteger(v)) {
            throw new Error(`unexpected ${sysfsBase}_calibration_accuracy value: ${raw}`);
        }
        return v;
    }

    /** Diagnostic: list discovered reports keyed by SHTP report id. */
    getReports() {
        const out = {};
        for (const [reportId, info] of this._reports) {
            out[reportId] = {
                reportId,
                key: info.meta.key,
                eventName: info.meta.eventName,
                kind: info.meta.kind,
                qShift: info.meta.qShift,
                axes: info.axes.map((a) => ({
                    sysfsBase: a.sysfsBase,
                    scanIndex: a.scanIndex,
                    type: a.type.raw,
                })),
            };
        }
        return out;
    }

    /** Diagnostic: list ids of reports whose axes currently have _en=1. */
    getEnabledReports() {
        const enabled = [];
        for (const [reportId, info] of this._reports) {
            if (info.axes.every((a) => this._readSysfs(a.enPath) === '1')) {
                enabled.push(reportId);
            }
        }
        return enabled;
    }

    close() {
        this._stopBuffer();
        if (this._fd !== null) {
            try { fs.closeSync(this._fd); } catch (err) { this._dbg(`close fd: ${err.message}`); }
            this._fd = null;
        }
    }

    [Symbol.dispose]() {
        this.close();
    }

    // -----------------------------------------------------------------
    // Internal buffer / poll lifecycle
    // -----------------------------------------------------------------

    _startBuffer() {
        const enabled = this.getEnabledReports();
        if (enabled.length === 0) {
            throw new Error('No reports enabled.');
        }

        // Timestamp must be on so we get a stable per-record tail.
        this._writeSysfs(this._timestamp.enPath, 1);

        if (this._bufferLength !== null && this._bufferLengthPath) {
            this._writeSysfs(this._bufferLengthPath, this._bufferLength);
        }

        this._writeSysfs(this._bufferEnablePath, 1);
        this._buffered = true;

        try {
            this._buildParsePlan(enabled);
        } catch (err) {
            this._writeSysfs(this._bufferEnablePath, 0);
            this._buffered = false;
            throw err;
        }

        this._drainDevice();

        if (!this._pollTimer) {
            this._pollTimer = setInterval(() => this._pollDevice(), this._pollIntervalMs);
        }
        this._dbg(`started: enabled=[${enabled.map((i) => '0x' + i.toString(16)).join(',')}] rec=${this._recordSize} poll=${this._pollIntervalMs}ms`);
    }

    _stopBuffer() {
        if (this._pollTimer) {
            clearInterval(this._pollTimer);
            this._pollTimer = null;
        }
        if (this._buffered) {
            try {
                this._writeSysfs(this._bufferEnablePath, 0);
            } catch (err) {
                this._dbg(`buffer disable failed: ${err.message}`);
            }
            this._buffered = false;
        }
        this._pending = Buffer.alloc(0);
    }

    // -----------------------------------------------------------------
    // Parse plan
    // -----------------------------------------------------------------

    _buildParsePlan(enabledIds) {
        // IIO mask order: ascending scan_index, each field self-aligned to its
        // element storage size, with a final pad to the largest alignment in
        // the record. Vec3 axes occupy three consecutive s32 slots that we
        // treat as one composite field anchored at the smallest scan_index of
        // the three (the kernel guarantees they are consecutive).
        const fields = [];

        for (const reportId of enabledIds) {
            const info = this._reports.get(reportId);
            if (!info) continue;
            const sortedAxes = [...info.axes].sort((a, b) => a.scanIndex - b.scanIndex);
            const firstAxis = sortedAxes[0];
            const wordCount = info.meta.kind === 'quat'
                ? firstAxis.type.repeat
                : sortedAxes.length;

            fields.push({
                kind: info.meta.kind,
                reportId,
                reportKey: info.meta.key,
                eventName: info.meta.eventName,
                qShift: info.meta.qShift,
                scanIndex: firstAxis.scanIndex,
                storageBytes: firstAxis.type.storageBytes,
                // The kernel treats a repeat=4 quaternion as one 16-byte element, not
                // four 4-byte ones; vec3 axes are separate channels and align to 4.
                elementBytes: info.meta.kind === 'quat'
                    ? firstAxis.type.storageBytes * firstAxis.type.repeat
                    : firstAxis.type.storageBytes,
                words: wordCount,
                totalBytes: firstAxis.type.storageBytes * wordCount,
            });
        }

        fields.push({
            kind: 'ts',
            reportId: null,
            scanIndex: this._timestamp.scanIndex,
            storageBytes: this._timestamp.type.storageBytes,
            elementBytes: this._timestamp.type.storageBytes,
            words: 1,
            totalBytes: this._timestamp.type.totalBytes,
        });

        fields.sort((a, b) => a.scanIndex - b.scanIndex);

        let offset = 0;
        let maxAlign = 1;
        for (const f of fields) {
            const align = f.elementBytes;
            if (align > maxAlign) maxAlign = align;
            offset = alignUp(offset, align);
            f.byteOffset = offset;
            offset += f.totalBytes;
        }
        const totalSize = alignUp(offset, maxAlign);

        if (totalSize < MIN_RECORD_SIZE || totalSize > MAX_RECORD_SIZE) {
            throw new Error(`Computed record size ${totalSize} outside [${MIN_RECORD_SIZE}, ${MAX_RECORD_SIZE}]`);
        }

        let kernelScanBytes = null;
        if (this._scanBytesPath) {
            const raw = this._readSysfs(this._scanBytesPath);
            const parsed = Number.parseInt(raw, 10);
            if (Number.isInteger(parsed) && parsed > 0) kernelScanBytes = parsed;
        }
        if (kernelScanBytes !== null && kernelScanBytes !== totalSize) {
            this._dbg(`WARN scan_bytes mismatch: kernel=${kernelScanBytes} computed=${totalSize}; using kernel value`);
            this._recordSize = kernelScanBytes;
        } else {
            this._recordSize = totalSize;
        }

        // iio_push_to_buffers_with_timestamp() puts the timestamp in the
        // last 8 bytes of the record, not sequentially.
        const tsField = fields.find((f) => f.kind === 'ts');
        if (tsField) {
            tsField.byteOffset = this._recordSize - tsField.totalBytes;
        }

        this._parsePlan = fields;

        if (this._debug) {
            for (const f of fields) {
                this._dbg(`plan ${f.reportKey || '__ts__'} kind=${f.kind} sidx=${f.scanIndex} off=${f.byteOffset} words=${f.words}`);
            }
            this._dbg(`record size = ${this._recordSize}`);
        }
    }

    // -----------------------------------------------------------------
    // Polling / decoding
    // -----------------------------------------------------------------

    _pollDevice() {
        if (this._fd === null) return;

        const readBuf = Buffer.alloc(this._recordSize * READ_BATCH_RECORDS);

        while (true) {
            let bytes = 0;
            try {
                bytes = fs.readSync(this._fd, readBuf, 0, readBuf.length, null);
            } catch (err) {
                if (err.code === 'EAGAIN' || err.code === 'EWOULDBLOCK') break;
                this.emit('error', err);
                break;
            }
            if (bytes <= 0) break;
            if ((bytes % this._recordSize) !== 0) {
                this._dbg(`WARN misaligned read=${bytes} rec=${this._recordSize}`);
            }
            this._pending = Buffer.concat([this._pending, readBuf.slice(0, bytes)]);
            this._consumePending();
        }
    }

    _drainDevice() {
        if (this._fd === null) return;
        const buf = Buffer.alloc(this._recordSize * READ_BATCH_RECORDS);
        for (let i = 0; i < 64; i += 1) {
            try {
                const n = fs.readSync(this._fd, buf, 0, buf.length, null);
                if (n <= 0) break;
            } catch (err) {
                if (err.code === 'EAGAIN' || err.code === 'EWOULDBLOCK') break;
                this.emit('error', err);
                break;
            }
        }
        this._pending = Buffer.alloc(0);
    }

    _consumePending() {
        while (this._pending.length >= this._recordSize) {
            const record = this._pending.slice(0, this._recordSize);
            this._pending = this._pending.slice(this._recordSize);
            this._handleRecord(record);
        }
    }

    _handleRecord(record) {
        this._recordsSeen += 1;

        // Pull timestamp first so each per-report event for this record shares it.
        let timestampNs = 0n;
        for (const f of this._parsePlan) {
            if (f.kind === 'ts') {
                timestampNs = record.readBigInt64LE(f.byteOffset);
                break;
            }
        }

        const logRaw = this._debugRaw && (
            this._debugRawMaxRecords <= 0 || this._recordsSeen <= this._debugRawMaxRecords
        );
        if (logRaw) {
            this._dbg(`rec#${this._recordsSeen} hex=${record.toString('hex')}`);
        }

        for (const f of this._parsePlan) {
            if (f.kind === 'ts') continue;

            const raw = new Array(f.words);
            for (let i = 0; i < f.words; i += 1) {
                raw[i] = record.readInt32LE(f.byteOffset + i * f.storageBytes);
            }

            const scale = f.qShift > 0 ? 1 / (1 << f.qShift) : 1;
            const data = raw.map((v) => v * scale);

            const extra = {
                kind: f.kind,
                raw,
                qShift: f.qShift,
                scale,
                timestampNs,
                recordSizeBytes: this._recordSize,
                byteOffset: f.byteOffset,
            };

            if (f.kind === 'quat') {
                const [qi, qj, qk, qr] = data;
                const norm = Math.sqrt(qi * qi + qj * qj + qk * qk + qr * qr);
                extra.norm = norm;
                extra.normLikelyValid = Math.abs(norm - 1) <= this._quaternionNormTolerance;
            }

            this._publish(f.reportId, data, { extra });
        }
    }

    // -----------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------

    _requireReport(reportId) {
        if (!Number.isInteger(reportId)) {
            throw new Error('feature_id must be a numeric BNO_REPORT_* id');
        }
        const info = this._reports.get(reportId);
        if (!info) {
            const available = [...this._reports.keys()].map((i) => `0x${i.toString(16)}`).join(',');
            throw new Error(`Report 0x${reportId.toString(16)} is not exposed by this IIO device (available: ${available || 'none'})`);
        }
        return info;
    }
}

function alignUp(value, alignment) {
    if (!Number.isInteger(alignment) || alignment <= 1) return value;
    return Math.ceil(value / alignment) * alignment;
}

function deepCopy(value) {
    if (Array.isArray(value)) return value.slice();
    if (value && typeof value === 'object') return { ...value };
    return value;
}

// SH2_CAL_* bits, mirrored from sh2/sh2.h. Used with
// {get,set}CalibrationAutoMask().
BNO08X_IIO.CAL_ACCEL    = 0x01;
BNO08X_IIO.CAL_GYRO     = 0x02;
BNO08X_IIO.CAL_MAG      = 0x04;
BNO08X_IIO.CAL_PLANAR   = 0x08;
BNO08X_IIO.CAL_ON_TABLE = 0x10;

module.exports = {
    BNO08X_IIO,
};
