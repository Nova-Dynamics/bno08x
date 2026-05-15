/**
 * SHTP (SH-2 Transport Protocol) helpers shared by the I2C transport.
 *
 * Holds packet framing, byte-level unpack/pack, the report parsers used by
 * the SHTP packet pump, and the SHTP channel/message-id constants. None of
 * this is needed by the IIO backend (the kernel driver does the SHTP work).
 */

const _BNO_HEADER_LEN = 4;
const DATA_BUFFER_SIZE = 512;

// SHTP channels
const BNO_CHANNEL_SHTP_COMMAND = 0;
const BNO_CHANNEL_EXE = 1;
const _BNO_CHANNEL_CONTROL = 2;
const _BNO_CHANNEL_INPUT_SENSOR_REPORTS = 3;
const _BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = 4;
const _BNO_CHANNEL_GYRO_ROTATION_VECTOR = 5;

// Control channel message ids
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

// DCD / ME calibration commands and sub-commands
const _SAVE_DCD = 0x6;
const _ME_CALIBRATE = 0x7;
const _ME_CAL_CONFIG = 0x00;
const _ME_GET_CAL = 0x01;

// Report-length table for control-channel messages (sensor report sizes are
// derived from the canonical REPORT_META in base.js).
const _CONTROL_REPORT_LENGTHS = {
    [_SHTP_REPORT_PRODUCT_ID_RESPONSE]: 16,
    [_GET_FEATURE_RESPONSE]: 17,
    [_COMMAND_RESPONSE]: 16,
    [_BASE_TIMESTAMP]: 5,
    [_TIMESTAMP_REBASE]: 5,
};

class PacketError extends Error {
    constructor(message) {
        super(message);
        this.name = 'PacketError';
    }
}

class PacketHeader {
    constructor(channel_number, sequence_number, data_length, packet_byte_count) {
        this.channel_number = channel_number;
        this.sequence_number = sequence_number;
        this.data_length = data_length;
        this.packet_byte_count = packet_byte_count;
    }
}

// ---------------------------------------------------------------------
// Byte helpers
// ---------------------------------------------------------------------

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

// ---------------------------------------------------------------------
// Report parsers
// ---------------------------------------------------------------------

/**
 * Parse a vec3/quat sensor report. Caller supplies the report metadata (scalar,
 * count, treat as raw/unsigned) so this stays decoupled from the canonical
 * report table in base.js.
 */
function _parse_sensor_report_data(report_bytes, { scalar, count, isRaw }) {
    const data_offset = 4;
    const accuracy = unpack_uint8(report_bytes, 2) & 0b11;

    const results = [];
    for (let offset_idx = 0; offset_idx < count; offset_idx++) {
        const total_offset = data_offset + (offset_idx * 2);
        const raw_data = isRaw
            ? unpack_uint16_le(report_bytes, total_offset)
            : unpack_int16_le(report_bytes, total_offset);
        results.push(raw_data * scalar);
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
        unpack_uint8(report_bytes, 0),
        unpack_uint8(report_bytes, 1),
        unpack_uint8(report_bytes, 2),
        unpack_uint16_le(report_bytes, 3),
        unpack_uint32_le(report_bytes, 5),
        unpack_uint32_le(report_bytes, 9),
        unpack_uint32_le(report_bytes, 13),
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

// ---------------------------------------------------------------------
// Packet framing
// ---------------------------------------------------------------------

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

/**
 * Walk a packet's payload and split it into report slices. `getSensorReportLength`
 * resolves the byte length of a sensor report id (decoupled from REPORT_META).
 */
function _separate_batch(packet, report_slices, getSensorReportLength) {
    let next_byte_index = 0;

    while (next_byte_index < packet.header.data_length) {
        const report_id = packet.data[next_byte_index];
        const required_bytes = report_id < 0xF0
            ? getSensorReportLength(report_id)
            : _CONTROL_REPORT_LENGTHS[report_id];
        if (!required_bytes) {
            throw new Error(`Unknown report id 0x${report_id.toString(16)} in batch`);
        }
        const unprocessed_byte_count = packet.header.data_length - next_byte_index;

        if (unprocessed_byte_count < required_bytes) {
            throw new Error(`Unprocessable Batch bytes: ${unprocessed_byte_count}`);
        }

        const report_slice = packet.data.slice(next_byte_index, next_byte_index + required_bytes);
        report_slices.push([report_slice[0], report_slice]);
        next_byte_index = next_byte_index + required_bytes;
    }
}

module.exports = {
    // Constants
    _BNO_HEADER_LEN,
    DATA_BUFFER_SIZE,
    BNO_CHANNEL_SHTP_COMMAND,
    BNO_CHANNEL_EXE,
    _BNO_CHANNEL_CONTROL,
    _BNO_CHANNEL_INPUT_SENSOR_REPORTS,
    _BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS,
    _BNO_CHANNEL_GYRO_ROTATION_VECTOR,
    _GET_FEATURE_REQUEST,
    _SET_FEATURE_COMMAND,
    _GET_FEATURE_RESPONSE,
    _BASE_TIMESTAMP,
    _TIMESTAMP_REBASE,
    _SHTP_REPORT_PRODUCT_ID_RESPONSE,
    _SHTP_REPORT_PRODUCT_ID_REQUEST,
    _FRS_WRITE_REQUEST,
    _FRS_WRITE_DATA,
    _FRS_WRITE_RESPONSE,
    _FRS_READ_REQUEST,
    _FRS_READ_RESPONSE,
    _COMMAND_REQUEST,
    _COMMAND_RESPONSE,
    _SAVE_DCD,
    _ME_CALIBRATE,
    _ME_CAL_CONFIG,
    _ME_GET_CAL,
    _CONTROL_REPORT_LENGTHS,

    // Classes
    Packet,
    PacketHeader,
    PacketError,

    // Byte helpers
    unpack_uint8,
    unpack_uint16_le,
    unpack_int16_le,
    unpack_uint32_le,
    pack_uint16_le,

    // Parsers / packers
    _parse_sensor_report_data,
    _parse_step_counter_report,
    _parse_stability_classifier_report,
    _parse_get_feature_response_report,
    _parse_activity_classifier_report,
    _parse_shake_report,
    _parse_command_response,
    _insert_command_request_report,
    parse_sensor_id,
    _separate_batch,
};
