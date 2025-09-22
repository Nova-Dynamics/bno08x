/**
 * BNO08x JavaScript library - Base class
 * 
 * Port of the adafruit_bno08x Python library for interfacing with the BNO08x IMU
 * Helper library for the Hillcrest Laboratories BNO08x IMUs
 * 
 * Original Python code by Bryan Siepert for Adafruit Industries
 * JavaScript port by Sean Sullivan
 */

const { EventEmitter } = require('events');

// Constants (equivalent to Python const() values)
const BNO_CHANNEL_SHTP_COMMAND = 0;
const BNO_CHANNEL_EXE = 1;
const _BNO_CHANNEL_CONTROL = 2;
const _BNO_CHANNEL_INPUT_SENSOR_REPORTS = 3;
const _BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = 4;
const _BNO_CHANNEL_GYRO_ROTATION_VECTOR = 5;

const _GET_FEATURE_REQUEST = 0xFE;
const _SET_FEATURE_COMMAND = 0xFD;
const _GET_FEATURE_RESPONSE = 0xFC;
const _BASE_TIMESTAMP = 0xFB;
const _TIMESTAMP_REBASE = 0xFA;

const _SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8;
const _SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9;

const _FRS_WRITE_REQUEST = 0xF7;
const _FRS_WRITE_DATA = 0xF6;
const _FRS_WRITE_RESPONSE = 0xF5;

const _FRS_READ_REQUEST = 0xF4;
const _FRS_READ_RESPONSE = 0xF3;

const _COMMAND_REQUEST = 0xF2;
const _COMMAND_RESPONSE = 0xF1;

// DCD/ ME Calibration commands and sub-commands
const _SAVE_DCD = 0x6;
const _ME_CALIBRATE = 0x7;
const _ME_CAL_CONFIG = 0x00;
const _ME_GET_CAL = 0x01;

// Report IDs
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

// Timing constants
const _DEFAULT_REPORT_INTERVAL = 50000; // in microseconds = 50ms
const _QUAT_READ_TIMEOUT = 0.500; // timeout in seconds
const _PACKET_READ_TIMEOUT = 2.000; // timeout in seconds
const _FEATURE_ENABLE_TIMEOUT = 2.0;
const _DEFAULT_TIMEOUT = 2.0;
const _BNO08X_CMD_RESET = 0x01;
const _QUAT_Q_POINT = 14;
const _BNO_HEADER_LEN = 4;

// Scaling factors
const _Q_POINT_14_SCALAR = Math.pow(2, -14);
const _Q_POINT_12_SCALAR = Math.pow(2, -12);
const _Q_POINT_9_SCALAR = Math.pow(2, -9);
const _Q_POINT_8_SCALAR = Math.pow(2, -8);
const _Q_POINT_4_SCALAR = Math.pow(2, -4);

const _GYRO_SCALAR = _Q_POINT_9_SCALAR;
const _ACCEL_SCALAR = _Q_POINT_8_SCALAR;
const _QUAT_SCALAR = _Q_POINT_14_SCALAR;
const _GEO_QUAT_SCALAR = _Q_POINT_12_SCALAR;
const _MAG_SCALAR = _Q_POINT_4_SCALAR;

// Report lengths
const _REPORT_LENGTHS = {
    [_SHTP_REPORT_PRODUCT_ID_RESPONSE]: 16,
    [_GET_FEATURE_RESPONSE]: 17,
    [_COMMAND_RESPONSE]: 16,
    [_BASE_TIMESTAMP]: 5,
    [_TIMESTAMP_REBASE]: 5,
};

// Raw reports mapping
const _RAW_REPORTS = {
    [BNO_REPORT_RAW_ACCELEROMETER]: BNO_REPORT_ACCELEROMETER,
    [BNO_REPORT_RAW_GYROSCOPE]: BNO_REPORT_GYROSCOPE,
    [BNO_REPORT_RAW_MAGNETOMETER]: BNO_REPORT_MAGNETOMETER,
};

// Available sensor reports
const _AVAIL_SENSOR_REPORTS = {
    [BNO_REPORT_ACCELEROMETER]: [_Q_POINT_8_SCALAR, 3, 10],
    [BNO_REPORT_GRAVITY]: [_Q_POINT_8_SCALAR, 3, 10],
    [BNO_REPORT_GYROSCOPE]: [_Q_POINT_9_SCALAR, 3, 10],
    [BNO_REPORT_MAGNETOMETER]: [_Q_POINT_4_SCALAR, 3, 10],
    [BNO_REPORT_LINEAR_ACCELERATION]: [_Q_POINT_8_SCALAR, 3, 10],
    [BNO_REPORT_ROTATION_VECTOR]: [_Q_POINT_14_SCALAR, 4, 14],
    [BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR]: [_Q_POINT_12_SCALAR, 4, 14],
    [BNO_REPORT_GAME_ROTATION_VECTOR]: [_Q_POINT_14_SCALAR, 4, 12],
    [BNO_REPORT_STEP_COUNTER]: [1, 1, 12],
    [BNO_REPORT_SHAKE_DETECTOR]: [1, 1, 6],
    [BNO_REPORT_STABILITY_CLASSIFIER]: [1, 1, 6],
    [BNO_REPORT_ACTIVITY_CLASSIFIER]: [1, 1, 16],
    [BNO_REPORT_RAW_ACCELEROMETER]: [1, 3, 16],
    [BNO_REPORT_RAW_GYROSCOPE]: [1, 3, 16],
    [BNO_REPORT_RAW_MAGNETOMETER]: [1, 3, 16],
};

// Initial reports
const _INITIAL_REPORTS = {
    [BNO_REPORT_ACTIVITY_CLASSIFIER]: {
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
    [BNO_REPORT_STABILITY_CLASSIFIER]: "Unknown",
    [BNO_REPORT_ROTATION_VECTOR]: [0.0, 0.0, 0.0, 0.0],
    [BNO_REPORT_GAME_ROTATION_VECTOR]: [0.0, 0.0, 0.0, 0.0],
    [BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR]: [0.0, 0.0, 0.0, 0.0],
};

const _ENABLED_ACTIVITIES = 0x1FF; // All activities; 1 bit set for each of 8 activities, + Unknown
const DATA_BUFFER_SIZE = 512; // data buffer size

const REPORT_ACCURACY_STATUS = [
    "Accuracy Unreliable",
    "Low Accuracy",
    "Medium Accuracy",
    "High Accuracy",
];

// Custom error class
class PacketError extends Error {
    constructor(message) {
        super(message);
        this.name = 'PacketError';
    }
}

// Utility functions
function _elapsed(start_time) {
    return Date.now() / 1000.0 - start_time;
}

// Packet header structure
class PacketHeader {
    constructor(channel_number, sequence_number, data_length, packet_byte_count) {
        this.channel_number = channel_number;
        this.sequence_number = sequence_number;
        this.data_length = data_length;
        this.packet_byte_count = packet_byte_count;
    }
}

// Helper functions for parsing binary data
function unpack_uint16_le(buffer, offset = 0) {
    return buffer[offset] | (buffer[offset + 1] << 8);
}

function unpack_int16_le(buffer, offset = 0) {
    const value = unpack_uint16_le(buffer, offset);
    return value > 32767 ? value - 65536 : value;
}

function unpack_uint8(buffer, offset = 0) {
    return buffer[offset];
}

function unpack_uint32_le(buffer, offset = 0) {
    return buffer[offset] | 
           (buffer[offset + 1] << 8) | 
           (buffer[offset + 2] << 16) | 
           (buffer[offset + 3] << 24);
}

function pack_uint16_le(value, buffer, offset = 0) {
    buffer[offset] = value & 0xFF;
    buffer[offset + 1] = (value >> 8) & 0xFF;
}

// Parsing functions
function _parse_sensor_report_data(report_bytes) {
    const data_offset = 4;
    const report_id = report_bytes[0];
    const [scalar, count, _report_length] = _AVAIL_SENSOR_REPORTS[report_id];
    
    const results = [];
    const accuracy = unpack_uint8(report_bytes, 2) & 0b11;
    
    for (let offset_idx = 0; offset_idx < count; offset_idx++) {
        const total_offset = data_offset + (offset_idx * 2);
        let raw_data;
        
        if (report_id in _RAW_REPORTS) {
            // raw reports are unsigned
            raw_data = unpack_uint16_le(report_bytes, total_offset);
        } else {
            raw_data = unpack_int16_le(report_bytes, total_offset);
        }
        
        const scaled_data = raw_data * scalar;
        results.push(scaled_data);
    }
    
    return [results, accuracy];
}

function _parse_step_counter_report(report_bytes) {
    return unpack_uint16_le(report_bytes, 8);
}

function _parse_stability_classifier_report(report_bytes) {
    const classification_bitfield = unpack_uint8(report_bytes, 4);
    const classifications = ["Unknown", "On Table", "Stationary", "Stable", "In motion"];
    return classifications[classification_bitfield] || "Unknown";
}

function _parse_get_feature_response_report(report_bytes) {
    return [
        unpack_uint8(report_bytes, 0),  // report_id
        unpack_uint8(report_bytes, 1),  // feature_report_id
        unpack_uint8(report_bytes, 2),  // feature_flags
        unpack_uint16_le(report_bytes, 3),  // change_sensitivity
        unpack_uint32_le(report_bytes, 5),  // report_interval
        unpack_uint32_le(report_bytes, 9),  // batch_interval_word
        unpack_uint32_le(report_bytes, 13), // sensor_specific_configuration_word
    ];
}

function _parse_activity_classifier_report(report_bytes) {
    const activities = [
        "Unknown",
        "In-Vehicle",
        "On-Bicycle",
        "On-Foot",
        "Still",
        "Tilting",
        "Walking",
        "Running",
        "OnStairs",
    ];
    
    const end_and_page_number = unpack_uint8(report_bytes, 4);
    const page_number = end_and_page_number & 0x7F;
    const most_likely = unpack_uint8(report_bytes, 5);
    
    const classification = {};
    classification["most_likely"] = activities[most_likely] || "Unknown";
    
    for (let idx = 0; idx < 9; idx++) {
        const raw_confidence = unpack_uint8(report_bytes, 6 + idx);
        const confidence = (10 * page_number) + raw_confidence;
        const activity_string = activities[idx];
        classification[activity_string] = confidence;
    }
    
    return classification;
}

function _parse_shake_report(report_bytes) {
    const shake_bitfield = unpack_uint16_le(report_bytes, 4);
    return (shake_bitfield & 0x111) > 0;
}

function parse_sensor_id(buffer) {
    if (buffer[0] !== _SHTP_REPORT_PRODUCT_ID_RESPONSE) {
        throw new Error(`Wrong report id for sensor id: 0x${buffer[0].toString(16)}`);
    }
    
    const sw_major = unpack_uint8(buffer, 2);
    const sw_minor = unpack_uint8(buffer, 3);
    const sw_patch = unpack_uint16_le(buffer, 12);
    const sw_part_number = unpack_uint32_le(buffer, 4);
    const sw_build_number = unpack_uint32_le(buffer, 8);
    
    return [sw_part_number, sw_major, sw_minor, sw_patch, sw_build_number];
}

function _parse_command_response(report_bytes) {
    const report_body = [
        unpack_uint8(report_bytes, 0),
        unpack_uint8(report_bytes, 1),
        unpack_uint8(report_bytes, 2),
        unpack_uint8(report_bytes, 3),
        unpack_uint8(report_bytes, 4),
    ];
    
    const response_values = [];
    for (let i = 0; i < 11; i++) {
        response_values.push(unpack_uint8(report_bytes, 5 + i));
    }
    
    return [report_body, response_values];
}

function _insert_command_request_report(command, buffer, next_sequence_number, command_params = null) {
    if (command_params && command_params.length > 9) {
        throw new Error(`Command request reports can only have up to 9 arguments but ${command_params.length} were given`);
    }
    
    // Clear buffer
    for (let i = 0; i < 12; i++) {
        buffer[i] = 0;
    }
    
    buffer[0] = _COMMAND_REQUEST;
    buffer[1] = next_sequence_number;
    buffer[2] = command;
    
    if (command_params) {
        for (let idx = 0; idx < command_params.length; idx++) {
            buffer[3 + idx] = command_params[idx];
        }
    }
}

function _report_length(report_id) {
    if (report_id < 0xF0) {
        // it's a sensor report
        return _AVAIL_SENSOR_REPORTS[report_id][2];
    }
    return _REPORT_LENGTHS[report_id];
}

function _separate_batch(packet, report_slices) {
    let next_byte_index = 0;
    
    while (next_byte_index < packet.header.data_length) {
        const report_id = packet.data[next_byte_index];
        const required_bytes = _report_length(report_id);
        const unprocessed_byte_count = packet.header.data_length - next_byte_index;
        
        // handle incomplete remainder
        if (unprocessed_byte_count < required_bytes) {
            throw new Error(`Unprocessable Batch bytes: ${unprocessed_byte_count}`);
        }
        
        // we have enough bytes to read
        const report_slice = packet.data.slice(next_byte_index, next_byte_index + required_bytes);
        report_slices.push([report_slice[0], report_slice]);
        next_byte_index = next_byte_index + required_bytes;
    }
}

// Packet class
class Packet {
    constructor(packet_bytes) {
        this.header = Packet.header_from_buffer(packet_bytes);
        const data_end_index = this.header.data_length + _BNO_HEADER_LEN;
        this.data = packet_bytes.slice(_BNO_HEADER_LEN, data_end_index);
    }
    
    get report_id() {
        return this.data[0];
    }
    
    get channel_number() {
        return this.header.channel_number;
    }
    
    static header_from_buffer(packet_bytes) {
        let packet_byte_count = unpack_uint16_le(packet_bytes, 0);
        packet_byte_count &= ~0x8000;
        const channel_number = unpack_uint8(packet_bytes, 2);
        const sequence_number = unpack_uint8(packet_bytes, 3);
        const data_length = Math.max(0, packet_byte_count - 4);
        
        return new PacketHeader(channel_number, sequence_number, data_length, packet_byte_count);
    }
    
    static is_error(header) {
        if (header.channel_number > 5) {
            return true;
        }
        if (header.packet_byte_count === 0xFFFF && header.sequence_number === 0xFF) {
            return true;
        }
        return false;
    }
    
    toString() {
        const length = this.header.packet_byte_count;
        let outstr = "\n\t\t********** Packet *************\n";
        outstr += "DBG::\t\t HEADER:\n";
        outstr += `DBG::\t\t Data Len: ${this.header.data_length}\n`;
        outstr += `DBG::\t\t Channel: ${this.channel_number}\n`;
        outstr += `DBG::\t\t Sequence number: ${this.header.sequence_number}\n`;
        outstr += "\n";
        outstr += "DBG::\t\t Data:";
        
        for (let idx = 0; idx < Math.min(this.data.length, length); idx++) {
            const packet_index = idx + 4;
            if ((packet_index % 4) === 0) {
                outstr += `\nDBG::\t\t[0x${packet_index.toString(16).padStart(2, '0').toUpperCase()}] `;
            }
            outstr += `0x${this.data[idx].toString(16).padStart(2, '0').toUpperCase()} `;
        }
        outstr += "\n";
        outstr += "\t\t*******************************\n";
        
        return outstr;
    }
}

// Main BNO08X class
class BNO08X extends EventEmitter {
    constructor(reset = null, debug = false) {
        super();
        this._debug = debug;
        this._reset = reset;
        this._dbg("********** __init__ *************");
        this._data_buffer = Buffer.alloc(DATA_BUFFER_SIZE);
        this._command_buffer = Buffer.alloc(12);
        this._packet_slices = [];
        
        // Sequence numbers for each channel
        this._sequence_number = [0, 0, 0, 0, 0, 0];
        this._two_ended_sequence_numbers = {};
        this._dcd_saved_at = -1;
        this._me_calibration_started_at = -1.0;
        this._calibration_complete = false;
        this._magnetometer_accuracy = 0;
        this._wait_for_initialize = true;
        this._init_complete = false;
        this._id_read = false;
        this._readings = {};
        
        // Initialize in next tick to allow subclass constructor to complete
        setImmediate(() => this.initialize());
    }
    
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
    
    // Properties
    get magnetic() {
        this._process_available_packets();
        if (!(BNO_REPORT_MAGNETOMETER in this._readings)) {
            throw new Error("No magfield report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_MAGNETOMETER];
    }
    
    get quaternion() {
        this._process_available_packets();
        if (!(BNO_REPORT_ROTATION_VECTOR in this._readings)) {
            throw new Error("No quaternion report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_ROTATION_VECTOR];
    }
    
    get geomagnetic_quaternion() {
        this._process_available_packets();
        if (!(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR in this._readings)) {
            throw new Error("No geomag quaternion report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR];
    }
    
    get game_quaternion() {
        this._process_available_packets();
        if (!(BNO_REPORT_GAME_ROTATION_VECTOR in this._readings)) {
            throw new Error("No game quaternion report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_GAME_ROTATION_VECTOR];
    }
    
    get steps() {
        this._process_available_packets();
        if (!(BNO_REPORT_STEP_COUNTER in this._readings)) {
            throw new Error("No steps report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_STEP_COUNTER];
    }
    
    get linear_acceleration() {
        this._process_available_packets();
        if (!(BNO_REPORT_LINEAR_ACCELERATION in this._readings)) {
            throw new Error("No lin. accel report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_LINEAR_ACCELERATION];
    }
    
    get acceleration() {
        this._process_available_packets();
        if (!(BNO_REPORT_ACCELEROMETER in this._readings)) {
            throw new Error("No accel report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_ACCELEROMETER];
    }
    
    get gravity() {
        this._process_available_packets();
        if (!(BNO_REPORT_GRAVITY in this._readings)) {
            throw new Error("No gravity report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_GRAVITY];
    }
    
    get gyro() {
        this._process_available_packets();
        if (!(BNO_REPORT_GYROSCOPE in this._readings)) {
            throw new Error("No gyro report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_GYROSCOPE];
    }
    
    get shake() {
        this._process_available_packets();
        if (!(BNO_REPORT_SHAKE_DETECTOR in this._readings)) {
            throw new Error("No shake report found, is it enabled?");
        }
        const shake_detected = this._readings[BNO_REPORT_SHAKE_DETECTOR];
        if (shake_detected) {
            this._readings[BNO_REPORT_SHAKE_DETECTOR] = false;
        }
        return shake_detected;
    }
    
    get stability_classification() {
        this._process_available_packets();
        if (!(BNO_REPORT_STABILITY_CLASSIFIER in this._readings)) {
            throw new Error("No stability classification report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_STABILITY_CLASSIFIER];
    }
    
    get activity_classification() {
        this._process_available_packets();
        if (!(BNO_REPORT_ACTIVITY_CLASSIFIER in this._readings)) {
            throw new Error("No activity classification report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_ACTIVITY_CLASSIFIER];
    }
    
    get raw_acceleration() {
        this._process_available_packets();
        if (!(BNO_REPORT_RAW_ACCELEROMETER in this._readings)) {
            throw new Error("No raw acceleration report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_RAW_ACCELEROMETER];
    }
    
    get raw_gyro() {
        this._process_available_packets();
        if (!(BNO_REPORT_RAW_GYROSCOPE in this._readings)) {
            throw new Error("No raw gyro report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_RAW_GYROSCOPE];
    }
    
    get raw_magnetic() {
        this._process_available_packets();
        if (!(BNO_REPORT_RAW_MAGNETOMETER in this._readings)) {
            throw new Error("No raw magnetic report found, is it enabled?");
        }
        return this._readings[BNO_REPORT_RAW_MAGNETOMETER];
    }
    
    // Methods
    begin_calibration() {
        this._send_me_command([
            1,  // calibrate accel
            1,  // calibrate gyro
            1,  // calibrate mag
            _ME_CAL_CONFIG,
            0,  // calibrate planar acceleration
            0,  // 'on_table' calibration
            0,  // reserved
            0,  // reserved
            0,  // reserved
        ]);
        this._calibration_complete = false;
    }
    
    get calibration_status() {
        this._send_me_command([
            0,  // calibrate accel
            0,  // calibrate gyro
            0,  // calibrate mag
            _ME_GET_CAL,
            0,  // calibrate planar acceleration
            0,  // 'on_table' calibration
            0,  // reserved
            0,  // reserved
            0,  // reserved
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
    
    // Private methods
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
            const new_packet = this._read_packet();
            return new_packet;
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
            _separate_batch(packet, this._packet_slices);
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
            const [_report_id, feature_report_id, ..._remainder] = get_feature_report;
            this._readings[feature_report_id] = _INITIAL_REPORTS[feature_report_id] || [0.0, 0.0, 0.0];
        }
        
        if (report_id === _COMMAND_RESPONSE) {
            this._handle_command_response(report_bytes);
        }
    }
    
    _handle_command_response(report_bytes) {
        const [report_body, response_values] = _parse_command_response(report_bytes);
        const [_report_id, _seq_number, command, _command_seq_number, _response_seq_number] = report_body;
        const [command_status, ..._rest] = response_values;
        
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
        
        if (report_id === BNO_REPORT_STEP_COUNTER) {
            const steps = _parse_step_counter_report(report_bytes);
            this._readings[report_id] = steps;
            this.emit('step_counter', steps);
            this.emit('report', { type: 'step_counter', data: steps, reportId: report_id });
            return;
        }
        
        if (report_id === BNO_REPORT_SHAKE_DETECTOR) {
            const shake_detected = _parse_shake_report(report_bytes);
            if (!(BNO_REPORT_SHAKE_DETECTOR in this._readings) || !this._readings[BNO_REPORT_SHAKE_DETECTOR]) {
                this._readings[BNO_REPORT_SHAKE_DETECTOR] = shake_detected;
            }
            if (shake_detected) {
                this.emit('shake', shake_detected);
                this.emit('report', { type: 'shake', data: shake_detected, reportId: report_id });
            }
            return;
        }
        
        if (report_id === BNO_REPORT_STABILITY_CLASSIFIER) {
            const stability_classification = _parse_stability_classifier_report(report_bytes);
            this._readings[BNO_REPORT_STABILITY_CLASSIFIER] = stability_classification;
            this.emit('stability_classification', stability_classification);
            this.emit('report', { type: 'stability_classification', data: stability_classification, reportId: report_id });
            return;
        }
        
        if (report_id === BNO_REPORT_ACTIVITY_CLASSIFIER) {
            const activity_classification = _parse_activity_classifier_report(report_bytes);
            this._readings[BNO_REPORT_ACTIVITY_CLASSIFIER] = activity_classification;
            this.emit('activity_classification', activity_classification);
            this.emit('report', { type: 'activity_classification', data: activity_classification, reportId: report_id });
            return;
        }
        
        const [sensor_data, accuracy] = _parse_sensor_report_data(report_bytes);
        if (report_id === BNO_REPORT_MAGNETOMETER) {
            this._magnetometer_accuracy = accuracy;
        }
        this._readings[report_id] = sensor_data;
        
        // Emit specific sensor events
        switch (report_id) {
            case BNO_REPORT_ACCELEROMETER:
                this.emit('accelerometer', sensor_data, accuracy);
                this.emit('report', { type: 'accelerometer', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_GYROSCOPE:
                this.emit('gyroscope', sensor_data, accuracy);
                this.emit('report', { type: 'gyroscope', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_MAGNETOMETER:
                this.emit('magnetometer', sensor_data, accuracy);
                this.emit('report', { type: 'magnetometer', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_LINEAR_ACCELERATION:
                this.emit('linear_acceleration', sensor_data, accuracy);
                this.emit('report', { type: 'linear_acceleration', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_ROTATION_VECTOR:
                this.emit('quaternion', sensor_data, accuracy);
                this.emit('report', { type: 'quaternion', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_GRAVITY:
                this.emit('gravity', sensor_data, accuracy);
                this.emit('report', { type: 'gravity', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_GAME_ROTATION_VECTOR:
                this.emit('game_quaternion', sensor_data, accuracy);
                this.emit('report', { type: 'game_quaternion', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR:
                this.emit('geomagnetic_quaternion', sensor_data, accuracy);
                this.emit('report', { type: 'geomagnetic_quaternion', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_RAW_ACCELEROMETER:
                this.emit('raw_accelerometer', sensor_data, accuracy);
                this.emit('report', { type: 'raw_accelerometer', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_RAW_GYROSCOPE:
                this.emit('raw_gyroscope', sensor_data, accuracy);
                this.emit('report', { type: 'raw_gyroscope', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_RAW_MAGNETOMETER:
                this.emit('raw_magnetometer', sensor_data, accuracy);
                this.emit('report', { type: 'raw_magnetometer', data: sensor_data, accuracy, reportId: report_id });
                break;
            case BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR:
                this.emit('gyro_integrated_rotation_vector', sensor_data, accuracy);
                this.emit('report', { type: 'gyro_integrated_rotation_vector', data: sensor_data, accuracy, reportId: report_id });
                break;
            default:
                // Emit generic sensor data event for any unhandled report types
                this.emit('sensor_data', { reportId: report_id, data: sensor_data, accuracy });
                this.emit('report', { type: 'unknown', data: sensor_data, accuracy, reportId: report_id });
                break;
        }
    }

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
    
    static _get_feature_enable_report(feature_id, report_interval = _DEFAULT_REPORT_INTERVAL, sensor_specific_config = 0) {
        const set_feature_report = Buffer.alloc(17);
        set_feature_report[0] = _SET_FEATURE_COMMAND;
        set_feature_report[1] = feature_id;
        
        // Pack report_interval as uint32 LE at offset 5
        set_feature_report.writeUInt32LE(report_interval, 5);
        
        // Pack sensor_specific_config as uint32 LE at offset 13
        set_feature_report.writeUInt32LE(sensor_specific_config, 13);
        
        return set_feature_report;
    }
    
    async enable_feature(feature_id, report_interval = _DEFAULT_REPORT_INTERVAL) {
        this._dbg(`\n********** Enabling feature id: ${feature_id} **********`);
        
        let set_feature_report;
        if (feature_id === BNO_REPORT_ACTIVITY_CLASSIFIER) {
            set_feature_report = BNO08X._get_feature_enable_report(
                feature_id, 
                report_interval, 
                _ENABLED_ACTIVITIES
            );
        } else {
            set_feature_report = BNO08X._get_feature_enable_report(feature_id, report_interval);
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
                return;
            }
            await this._sleep(0.01);
        }
        throw new Error(`Was not able to enable feature ${feature_id}`);
    }
    
    async _check_id() {
        this._dbg("\n********** READ ID **********");
        if (this._id_read) {
            return true;
        }
        
        const data = Buffer.alloc(2);
        data[0] = _SHTP_REPORT_PRODUCT_ID_REQUEST;
        data[1] = 0; // padding
        
        this._dbg("\n** Sending ID Request Report **");
        this._send_packet(_BNO_CHANNEL_CONTROL, data);
        this._dbg("\n** Waiting for packet **");
        
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
    
    _dbg(...args) {
        if (this._debug) {
            console.log("DBG::\t\t", ...args);
        }
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
    
    get _data_ready() {
        throw new Error("Not implemented - must be overridden by subclass");
    }
    
    hard_reset() {
        if (!this._reset) {
            return;
        }
        // TODO: Implement GPIO control for hardware reset
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
    
    _send_packet(channel, data) {
        throw new Error("Not implemented - must be overridden by subclass");
    }
    
    _read_packet() {
        throw new Error("Not implemented - must be overridden by subclass");
    }
    
    _increment_report_seq(report_id) {
        const current = this._two_ended_sequence_numbers[report_id] || 0;
        this._two_ended_sequence_numbers[report_id] = (current + 1) % 256;
    }
    
    _get_report_seq_id(report_id) {
        return this._two_ended_sequence_numbers[report_id] || 0;
    }
    
    // Helper sleep function
    async _sleep(seconds) {
        return new Promise(resolve => setTimeout(resolve, seconds * 1000));
    }
    
    _sleep_sync(seconds) {
        // Use busy wait for synchronous sleep (not ideal but sometimes necessary)
        const start = Date.now();
        while (Date.now() - start < seconds * 1000) {
            // busy wait
        }
    }
}

module.exports = {
    BNO08X,
    Packet,
    PacketError,
    PacketHeader,
    // Constants
    BNO_CHANNEL_SHTP_COMMAND,
    BNO_CHANNEL_EXE,
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
    REPORT_ACCURACY_STATUS,
    // Helper functions
    parse_sensor_id,
    _parse_sensor_report_data,
    _parse_step_counter_report,
    _parse_stability_classifier_report,
    _parse_activity_classifier_report,
    _parse_shake_report,
};
