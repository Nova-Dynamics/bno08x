/**
 * BNO08x I2C transport.
 *
 * Owns the SHTP packet pump, sequence numbers, calibration commands, feature
 * enable handshake, identification, soft/hard reset, and the auto-poll loop.
 * Decoded sensor reports are forwarded to the shared engine via _publish().
 */

const i2c = require('i2c-bus');

const {
    BNO08X,
    REPORT_META_BY_ID,
    getSensorReportLength,
    _RAW_REPORTS,
    _DEFAULT_REPORT_INTERVAL,
    _FEATURE_ENABLE_TIMEOUT,
    _DEFAULT_TIMEOUT,
    _PACKET_READ_TIMEOUT,
    _ENABLED_ACTIVITIES,
    _elapsed,

    // Report ids for the special-case parsers
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_SHAKE_DETECTOR,
    BNO_REPORT_STABILITY_CLASSIFIER,
    BNO_REPORT_ACTIVITY_CLASSIFIER,
    BNO_REPORT_MAGNETOMETER,
} = require('./base');

const {
    Packet,
    PacketError,
    PacketHeader,
    DATA_BUFFER_SIZE,

    BNO_CHANNEL_SHTP_COMMAND,
    BNO_CHANNEL_EXE,
    _BNO_CHANNEL_CONTROL,
    _SET_FEATURE_COMMAND,
    _GET_FEATURE_RESPONSE,
    _SHTP_REPORT_PRODUCT_ID_REQUEST,
    _SHTP_REPORT_PRODUCT_ID_RESPONSE,
    _COMMAND_REQUEST,
    _COMMAND_RESPONSE,
    _SAVE_DCD,
    _ME_CALIBRATE,
    _ME_CAL_CONFIG,
    _ME_GET_CAL,

    parse_sensor_id,
    _parse_sensor_report_data,
    _parse_step_counter_report,
    _parse_stability_classifier_report,
    _parse_activity_classifier_report,
    _parse_shake_report,
    _parse_get_feature_response_report,
    _parse_command_response,
    _insert_command_request_report,
    _separate_batch,
} = require('./sh2');

const _BNO08X_DEFAULT_ADDRESS = 0x4A;

class BNO08X_I2C extends BNO08X {
    /**
     * @param {number} i2c_bus_number  e.g. 1 for /dev/i2c-1
     * @param {Object} [options]
     * @param {number} [options.address=0x4A]
     * @param {boolean} [options.debug=false]
     * @param {*}      [options.reset=null]   reset pin control (not implemented)
     * @param {number|null} [options.autoReportIntervalUs=null]  If set, automatically starts the
     *   poll timer after enable_feature() is called (value in microseconds, e.g. 1000 = 1 ms).
     *   Mirrors the IIO backend's automatic polling behaviour so callers don't need to call
     *   start_auto_reporting() manually.
     */
    constructor(i2c_bus_number, options = {}) {
        const {
            address = _BNO08X_DEFAULT_ADDRESS,
            debug = false,
            reset = null,
            autoReportIntervalUs = null,
        } = options;

        super({ debug });

        this._dbg("********** __init__ *************");
        this._reset = reset;
        this._autoReportIntervalUs = autoReportIntervalUs;
        this._bus_number = i2c_bus_number;
        this._address = address;
        this._bus = null;

        this._data_buffer = Buffer.alloc(DATA_BUFFER_SIZE);
        this._command_buffer = Buffer.alloc(12);
        this._packet_slices = [];
        this._sequence_number = [0, 0, 0, 0, 0, 0];
        this._two_ended_sequence_numbers = {};
        this._dcd_saved_at = -1;
        this._me_calibration_started_at = -1.0;
        this._calibration_complete = false;
        this._magnetometer_accuracy = 0;
        this._wait_for_initialize = true;
        this._init_complete = false;
        this._id_read = false;

        try {
            this._bus = i2c.openSync(this._bus_number);
            this._dbg(`Opened I2C bus ${this._bus_number} at address 0x${this._address.toString(16)}`);
        } catch (error) {
            throw new Error(`Failed to open I2C bus ${this._bus_number}: ${error.message}`);
        }

        // Defer initialize() so subclassing finishes before we touch the bus.
        setImmediate(() => this.initialize());
    }

    // -----------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------

    async initialize() {
        for (let attempt = 0; attempt < 3; attempt++) {
            this.hard_reset();
            this.soft_reset();
            try {
                if (await this._check_id()) {
                    break;
                }
            } catch (error) {
                await this._sleep(0.5);
            }
        }
        if (!this._id_read) {
            throw new Error("Could not read ID");
        }
    }

    close() {
        this.stop_auto_reporting();
        if (this._bus) {
            try {
                this._bus.closeSync();
                this._dbg("Closed I2C bus");
            } catch (error) {
                this._dbg(`Error closing I2C bus: ${error.message}`);
            } finally {
                this._bus = null;
            }
        }
    }

    [Symbol.dispose]() {
        this.close();
    }

    // -----------------------------------------------------------------
    // Engine hook
    // -----------------------------------------------------------------

    _refreshCache() {
        this._process_available_packets();
    }

    // -----------------------------------------------------------------
    // Calibration
    // -----------------------------------------------------------------

    begin_calibration() {
        this._send_me_command([
            1, 1, 1,
            _ME_CAL_CONFIG,
            0, 0, 0, 0, 0,
        ]);
        this._calibration_complete = false;
    }

    get calibration_status() {
        this._send_me_command([
            0, 0, 0,
            _ME_GET_CAL,
            0, 0, 0, 0, 0,
        ]);
        return this._magnetometer_accuracy;
    }

    async _send_me_command(subcommand_params = null) {
        const start_time = Date.now() / 1000.0;
        const local_buffer = Buffer.alloc(12);
        _insert_command_request_report(
            _ME_CALIBRATE,
            local_buffer,
            this._get_report_seq_id(_COMMAND_REQUEST),
            subcommand_params
        );
        this._send_packet(_BNO_CHANNEL_CONTROL, local_buffer);
        this._increment_report_seq(_COMMAND_REQUEST);

        while (_elapsed(start_time) < _DEFAULT_TIMEOUT) {
            this._process_available_packets();
            if (this._me_calibration_started_at > start_time) {
                break;
            }
            await this._sleep(0.01);
        }
    }

    async save_calibration_data() {
        const start_time = Date.now() / 1000.0;
        const local_buffer = Buffer.alloc(12);
        _insert_command_request_report(
            _SAVE_DCD,
            local_buffer,
            this._get_report_seq_id(_COMMAND_REQUEST)
        );
        this._send_packet(_BNO_CHANNEL_CONTROL, local_buffer);
        this._increment_report_seq(_COMMAND_REQUEST);

        while (_elapsed(start_time) < _DEFAULT_TIMEOUT) {
            this._process_available_packets();
            if (this._dcd_saved_at > start_time) {
                return;
            }
            await this._sleep(0.01);
        }
        throw new Error("Could not save calibration data");
    }

    // -----------------------------------------------------------------
    // Feature enable
    // -----------------------------------------------------------------

    static _get_feature_enable_report(feature_id, report_interval = _DEFAULT_REPORT_INTERVAL, sensor_specific_config = 0) {
        const set_feature_report = Buffer.alloc(17);
        set_feature_report[0] = _SET_FEATURE_COMMAND;
        set_feature_report[1] = feature_id;
        set_feature_report.writeUInt32LE(report_interval, 5);
        set_feature_report.writeUInt32LE(sensor_specific_config, 13);
        return set_feature_report;
    }

    async enable_feature(feature_id, report_interval = _DEFAULT_REPORT_INTERVAL) {
        this._dbg(`\n********** Enabling feature id: ${feature_id} **********`);

        let set_feature_report;
        if (feature_id === BNO_REPORT_ACTIVITY_CLASSIFIER) {
            set_feature_report = BNO08X_I2C._get_feature_enable_report(
                feature_id,
                report_interval,
                _ENABLED_ACTIVITIES
            );
        } else {
            set_feature_report = BNO08X_I2C._get_feature_enable_report(feature_id, report_interval);
        }

        const feature_dependency = _RAW_REPORTS[feature_id];
        if (feature_dependency && !(feature_dependency in this._readings)) {
            this._dbg(`Enabling feature dependency: ${feature_dependency}`);
            await this.enable_feature(feature_dependency);
        }

        this._dbg(`Enabling ${feature_id}`);
        this._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report);

        const start_time = Date.now() / 1000.0;
        while (_elapsed(start_time) < _FEATURE_ENABLE_TIMEOUT) {
            this._process_available_packets(10);
            if (feature_id in this._readings) {
                if (this._autoReportIntervalUs != null && !this.auto_report_interval) {
                    this.start_auto_reporting(this._autoReportIntervalUs);
                }
                return;
            }
            await this._sleep(0.01);
        }
        throw new Error(`Was not able to enable feature ${feature_id}`);
    }

    // -----------------------------------------------------------------
    // Auto polling
    // -----------------------------------------------------------------

    async start_auto_reporting(report_interval = _DEFAULT_REPORT_INTERVAL) {
        this.auto_report_interval = setInterval(() => {
            this._process_available_packets();
        }, report_interval / 1000.0);
    }

    stop_auto_reporting() {
        if (this.auto_report_interval) {
            clearInterval(this.auto_report_interval);
            this.auto_report_interval = null;
        }
    }

    // -----------------------------------------------------------------
    // Reset
    // -----------------------------------------------------------------

    hard_reset() {
        if (!this._reset) {
            return;
        }
        // TODO: Implement GPIO control for hardware reset.
        console.log("Hardware reset not implemented - no GPIO control available");
    }

    soft_reset() {
        this._dbg("Soft resetting...");
        const data = Buffer.alloc(1);
        data[0] = 1;
        this._send_packet(BNO_CHANNEL_EXE, data);
        this._sleep_sync(0.5);
        this._send_packet(BNO_CHANNEL_EXE, data);
        this._sleep_sync(0.5);

        for (let i = 0; i < 3; i++) {
            try {
                this._read_packet();
            } catch (error) {
                if (error instanceof PacketError) {
                    this._sleep_sync(0.5);
                }
            }
        }

        this._dbg("OK!");
    }

    // -----------------------------------------------------------------
    // Packet pump
    // -----------------------------------------------------------------

    _process_available_packets(max_packets = null) {
        let processed_count = 0;
        while (this._data_ready) {
            if (max_packets && processed_count > max_packets) {
                return;
            }
            try {
                const new_packet = this._read_packet();
                this._handle_packet(new_packet);
                processed_count++;
            } catch (error) {
                if (error instanceof PacketError) {
                    continue;
                }
                throw error;
            }
        }
        this._dbg("** DONE! **");
    }

    async _wait_for_packet_type(channel_number, report_id = null, timeout = 5.0) {
        const report_id_str = report_id ? ` with report id 0x${report_id.toString(16)}` : "";
        this._dbg(`** Waiting for packet on channel ${channel_number}${report_id_str}`);

        const start_time = Date.now() / 1000.0;
        while (_elapsed(start_time) < timeout) {
            const new_packet = await this._wait_for_packet();

            if (new_packet.channel_number === channel_number) {
                if (report_id) {
                    if (new_packet.report_id === report_id) {
                        return new_packet;
                    }
                } else {
                    return new_packet;
                }
            }

            if (![BNO_CHANNEL_EXE, BNO_CHANNEL_SHTP_COMMAND].includes(new_packet.channel_number)) {
                this._dbg("passing packet to handler for de-slicing");
                this._handle_packet(new_packet);
            }

            await this._sleep(0.01);
        }

        throw new Error(`Timed out waiting for a packet on channel ${channel_number}`);
    }

    async _wait_for_packet(timeout = _PACKET_READ_TIMEOUT) {
        const start_time = Date.now() / 1000.0;
        while (_elapsed(start_time) < timeout) {
            if (!this._data_ready) {
                await this._sleep(0.01);
                continue;
            }
            return this._read_packet();
        }
        throw new Error("Timed out waiting for a packet");
    }

    _update_sequence_number(new_packet) {
        const channel = new_packet.channel_number;
        const seq = new_packet.header.sequence_number;
        this._sequence_number[channel] = seq;
    }

    _handle_packet(packet) {
        try {
            _separate_batch(packet, this._packet_slices, getSensorReportLength);
            while (this._packet_slices.length > 0) {
                const [report_id, report_bytes] = this._packet_slices.pop();
                this._process_report(report_id, report_bytes);
            }
        } catch (error) {
            console.log(packet.toString());
            throw error;
        }
    }

    _handle_control_report(report_id, report_bytes) {
        if (report_id === _SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            const [sw_part_number, sw_major, sw_minor, sw_patch, sw_build_number] = parse_sensor_id(report_bytes);
            this._dbg("FROM PACKET SLICE:");
            this._dbg(`*** Part Number: ${sw_part_number}`);
            this._dbg(`*** Software Version: ${sw_major}.${sw_minor}.${sw_patch}`);
            this._dbg(`\tBuild: ${sw_build_number}`);
            this._dbg("");
        }

        if (report_id === _GET_FEATURE_RESPONSE) {
            const get_feature_report = _parse_get_feature_response_report(report_bytes);
            const [, feature_report_id] = get_feature_report;
            const meta = REPORT_META_BY_ID[feature_report_id];
            // Seed the readings cache so enable_feature() knows the feature is live.
            this._readings[feature_report_id] = meta
                ? deepCopy(meta.initial)
                : [0.0, 0.0, 0.0];
        }

        if (report_id === _COMMAND_RESPONSE) {
            this._handle_command_response(report_bytes);
        }
    }

    _handle_command_response(report_bytes) {
        const [report_body, response_values] = _parse_command_response(report_bytes);
        const [, , command] = report_body;
        const [command_status] = response_values;

        if (command === _ME_CALIBRATE && command_status === 0) {
            this._me_calibration_started_at = Date.now() / 1000.0;
        }

        if (command === _SAVE_DCD) {
            if (command_status === 0) {
                this._dcd_saved_at = Date.now() / 1000.0;
            } else {
                throw new Error("Unable to save calibration data");
            }
        }
    }

    _process_report(report_id, report_bytes) {
        if (report_id >= 0xF0) {
            this._handle_control_report(report_id, report_bytes);
            return;
        }

        this._dbg(`\tProcessing report: 0x${report_id.toString(16)}`);

        // Special-case reports that don't follow the vec3/quat layout.
        if (report_id === BNO_REPORT_STEP_COUNTER) {
            const steps = _parse_step_counter_report(report_bytes);
            this._publish(report_id, steps);
            return;
        }

        if (report_id === BNO_REPORT_SHAKE_DETECTOR) {
            const shake_detected = _parse_shake_report(report_bytes);
            // Preserve "sticky" semantics: once we see a shake, leave it true
            // in the cache until a getter reads it (cleared in BNO08X.shake).
            const prev = this._readings[BNO_REPORT_SHAKE_DETECTOR];
            const next = (prev === undefined || !prev) ? shake_detected : prev;
            // Always update cache; only emit on a positive detection.
            this._readings[BNO_REPORT_SHAKE_DETECTOR] = next;
            if (shake_detected) {
                this._publish(report_id, shake_detected);
            }
            return;
        }

        if (report_id === BNO_REPORT_STABILITY_CLASSIFIER) {
            const stability = _parse_stability_classifier_report(report_bytes);
            this._publish(report_id, stability);
            return;
        }

        if (report_id === BNO_REPORT_ACTIVITY_CLASSIFIER) {
            const activity = _parse_activity_classifier_report(report_bytes);
            this._publish(report_id, activity);
            return;
        }

        const meta = REPORT_META_BY_ID[report_id];
        if (!meta) {
            this._dbg(`Unknown sensor report id: 0x${report_id.toString(16)}`);
            return;
        }
        const [sensor_data, accuracy] = _parse_sensor_report_data(report_bytes, {
            scalar: meta.scalar,
            count: meta.axisCount,
            isRaw: meta.isRaw,
        });
        if (report_id === BNO_REPORT_MAGNETOMETER) {
            this._magnetometer_accuracy = accuracy;
        }
        this._publish(report_id, sensor_data, { accuracy });
    }

    // -----------------------------------------------------------------
    // Identification
    // -----------------------------------------------------------------

    async _check_id() {
        this._dbg("\n********** READ ID **********");
        if (this._id_read) {
            return true;
        }

        const data = Buffer.alloc(2);
        data[0] = _SHTP_REPORT_PRODUCT_ID_REQUEST;
        data[1] = 0;

        this._dbg("\n** Sending ID Request Report **");
        this._send_packet(_BNO_CHANNEL_CONTROL, data);
        this._dbg("\n** Waiting for packet **");

        // eslint-disable-next-line no-constant-condition
        while (true) {
            await this._wait_for_packet_type(_BNO_CHANNEL_CONTROL, _SHTP_REPORT_PRODUCT_ID_RESPONSE);
            const sensor_id = this._parse_sensor_id();
            if (sensor_id) {
                this._id_read = true;
                return true;
            }
            this._dbg("Packet didn't have sensor ID report, trying again");
        }
    }

    _parse_sensor_id() {
        if (this._data_buffer[4] !== _SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            return null;
        }

        const sw_major = this._get_data(2, 'uint8');
        const sw_minor = this._get_data(3, 'uint8');
        const sw_patch = this._get_data(12, 'uint16le');
        const sw_part_number = this._get_data(4, 'uint32le');
        const sw_build_number = this._get_data(8, 'uint32le');

        this._dbg("");
        this._dbg(`*** Part Number: ${sw_part_number}`);
        this._dbg(`*** Software Version: ${sw_major}.${sw_minor}.${sw_patch}`);
        this._dbg(` Build: ${sw_build_number}`);
        this._dbg("");

        return sw_part_number;
    }

    _get_data(index, format) {
        const data_index = index + 4;
        switch (format) {
            case 'uint8':
                return this._data_buffer[data_index];
            case 'uint16le':
                return this._data_buffer.readUInt16LE(data_index);
            case 'uint32le':
                return this._data_buffer.readUInt32LE(data_index);
            default:
                throw new Error(`Unsupported format: ${format}`);
        }
    }

    _increment_report_seq(report_id) {
        const current = this._two_ended_sequence_numbers[report_id] || 0;
        this._two_ended_sequence_numbers[report_id] = (current + 1) % 256;
    }

    _get_report_seq_id(report_id) {
        return this._two_ended_sequence_numbers[report_id] || 0;
    }

    // -----------------------------------------------------------------
    // I2C transport
    // -----------------------------------------------------------------

    _send_packet(channel, data) {
        const data_length = data.length;
        const write_length = data_length + 4;

        this._data_buffer.writeUInt16LE(write_length, 0);
        this._data_buffer[2] = channel;
        this._data_buffer[3] = this._sequence_number[channel];

        for (let idx = 0; idx < data_length; idx++) {
            this._data_buffer[4 + idx] = data[idx];
        }

        const packet = new Packet(this._data_buffer);
        this._dbg("Sending packet:");
        this._dbg(packet.toString());

        try {
            this._bus.i2cWriteSync(
                this._address,
                write_length,
                this._data_buffer.slice(0, write_length)
            );
        } catch (error) {
            throw new Error(`I2C write failed: ${error.message}`);
        }

        this._sequence_number[channel] = (this._sequence_number[channel] + 1) % 256;
        return this._sequence_number[channel];
    }

    _read_header() {
        try {
            const header_data = Buffer.alloc(4);
            this._bus.i2cReadSync(this._address, 4, header_data);
            header_data.copy(this._data_buffer, 0);
        } catch (error) {
            throw new Error(`I2C header read failed: ${error.message}`);
        }

        const packet_header = Packet.header_from_buffer(this._data_buffer);
        this._dbg(packet_header);
        return packet_header;
    }

    _read_packet() {
        try {
            const header_data = Buffer.alloc(4);
            this._bus.i2cReadSync(this._address, 4, header_data);
            header_data.copy(this._data_buffer, 0);
        } catch (error) {
            throw new Error(`I2C packet header read failed: ${error.message}`);
        }

        this._dbg("");

        const header = Packet.header_from_buffer(this._data_buffer);
        const packet_byte_count = header.packet_byte_count;
        const channel_number = header.channel_number;
        const sequence_number = header.sequence_number;

        this._sequence_number[channel_number] = sequence_number;

        if (packet_byte_count === 0) {
            this._dbg("SKIPPING NO PACKETS AVAILABLE in i2c._read_packet");
            throw new PacketError("No packet available");
        }

        const data_bytes_to_read = packet_byte_count - 4;
        this._dbg(`channel ${channel_number} has ${data_bytes_to_read} bytes available to read`);

        this._read(data_bytes_to_read);

        const new_packet = new Packet(this._data_buffer);
        if (this._debug) {
            console.log(new_packet.toString());
        }

        this._update_sequence_number(new_packet);
        return new_packet;
    }

    _read(requested_read_length) {
        this._dbg(`trying to read ${requested_read_length} bytes`);
        const total_read_length = requested_read_length + 4;

        if (total_read_length > DATA_BUFFER_SIZE) {
            this._data_buffer = Buffer.alloc(total_read_length);
            this._dbg(`!!!!!!!!!!!! ALLOCATION: increased _data_buffer to ${total_read_length} bytes !!!!!!!!!!!!! `);
        }

        try {
            const read_data = Buffer.alloc(total_read_length);
            this._bus.i2cReadSync(this._address, total_read_length, read_data);
            read_data.copy(this._data_buffer, 0);
        } catch (error) {
            throw new Error(`I2C read failed: ${error.message}`);
        }
    }

    get _data_ready() {
        const header = this._read_header();

        if (header.channel_number > 5) {
            this._dbg(`channel number out of range: ${header.channel_number}`);
            return false;
        }

        if (header.packet_byte_count === 0x7FFF) {
            console.log("Byte count is 0x7FFF/0xFFFF; Error?");
            if (header.sequence_number === 0xFF) {
                console.log("Sequence number is 0xFF; Error?");
            }
            return false;
        }

        return header.data_length > 0;
    }
}

function deepCopy(value) {
    if (Array.isArray(value)) return value.slice();
    if (value && typeof value === 'object') return { ...value };
    return value;
}

module.exports = {
    BNO08X_I2C,
    _BNO08X_DEFAULT_ADDRESS,
};
