/**
 * BNO08x I2C implementation for Node.js
 * 
 * Subclass of BNO08X to use I2C communication
 * Uses the i2c-bus library for I2C control
 */

const i2c = require('i2c-bus');
const { BNO08X, Packet, PacketError } = require('./base');

const _BNO08X_DEFAULT_ADDRESS = 0x4A;
const DATA_BUFFER_SIZE = 512;

class BNO08X_I2C extends BNO08X {
    /**
     * Library for the BNO08x IMUs from Hillcrest Laboratories
     * 
     * @param {number} i2c_bus_number - The I2C bus number (e.g., 1 for /dev/i2c-1)
     * @param {Object} options - Configuration options
     * @param {number} options.address - I2C address (default: 0x4A)
     * @param {boolean} options.debug - Enable debug output (default: false)
     * @param {*} options.reset - Reset pin control (not implemented)
     */
    constructor(i2c_bus_number, options = {}) {
        const {
            address = _BNO08X_DEFAULT_ADDRESS,
            debug = false,
            reset = null
        } = options;
        
        super(reset, debug);
        
        this._bus_number = i2c_bus_number;
        this._address = address;
        this._bus = null;
        
        // Open I2C bus
        try {
            this._bus = i2c.openSync(this._bus_number);
            this._dbg(`Opened I2C bus ${this._bus_number} at address 0x${this._address.toString(16)}`);
        } catch (error) {
            throw new Error(`Failed to open I2C bus ${this._bus_number}: ${error.message}`);
        }
    }
    
    /**
     * Close the I2C bus connection
     */
    close() {
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
    
    /**
     * Send a packet via I2C
     * @param {number} channel - Channel number
     * @param {Buffer} data - Data to send
     * @returns {number} - Sequence number
     */
    _send_packet(channel, data) {
        const data_length = data.length;
        const write_length = data_length + 4;
        
        // Prepare packet with header
        this._data_buffer.writeUInt16LE(write_length, 0);
        this._data_buffer[2] = channel;
        this._data_buffer[3] = this._sequence_number[channel];
        
        // Copy data
        for (let idx = 0; idx < data_length; idx++) {
            this._data_buffer[4 + idx] = data[idx];
        }
        
        const packet = new Packet(this._data_buffer);
        this._dbg("Sending packet:");
        this._dbg(packet.toString());
        
        try {
            // Write to I2C device - BNO08x uses direct write without register addressing
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
    
    /**
     * Read the first 4 bytes available as a header
     * @returns {Object} - Packet header
     */
    _read_header() {
        try {
            const header_data = Buffer.alloc(4);
            this._bus.i2cReadSync(
                this._address,
                4,
                header_data
            );
            // Copy to data buffer
            header_data.copy(this._data_buffer, 0);
        } catch (error) {
            throw new Error(`I2C header read failed: ${error.message}`);
        }
        
        const packet_header = Packet.header_from_buffer(this._data_buffer);
        this._dbg(packet_header);
        return packet_header;
    }
    
    /**
     * Read a complete packet from I2C
     * @returns {Packet} - The received packet
     */
    _read_packet() {
        try {
            // Read header first
            const header_data = Buffer.alloc(4);
            this._bus.i2cReadSync(
                this._address,
                4,
                header_data
            );
            // Copy to data buffer
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
        this._dbg(
            `channel ${channel_number} has ${data_bytes_to_read} bytes available to read`
        );
        
        this._read(data_bytes_to_read);
        
        const new_packet = new Packet(this._data_buffer);
        if (this._debug) {
            console.log(new_packet.toString());
        }
        
        this._update_sequence_number(new_packet);
        
        return new_packet;
    }
    
    /**
     * Read the specified number of bytes from I2C
     * @param {number} requested_read_length - Number of bytes to read
     */
    _read(requested_read_length) {
        this._dbg(`trying to read ${requested_read_length} bytes`);
        
        // +4 for the header
        const total_read_length = requested_read_length + 4;
        
        if (total_read_length > DATA_BUFFER_SIZE) {
            this._data_buffer = Buffer.alloc(total_read_length);
            this._dbg(
                `!!!!!!!!!!!! ALLOCATION: increased _data_buffer to ${total_read_length} bytes !!!!!!!!!!!!! `
            );
        }
        
        try {
            const read_data = Buffer.alloc(total_read_length);
            this._bus.i2cReadSync(
                this._address,
                total_read_length,
                read_data
            );
            // Copy to data buffer
            read_data.copy(this._data_buffer, 0);
        } catch (error) {
            throw new Error(`I2C read failed: ${error.message}`);
        }
    }
    
    /**
     * Check if data is ready to be read
     * @returns {boolean} - True if data is ready
     */
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
        
        const ready = header.data_length > 0;
        return ready;
    }
    
    /**
     * Destructor - ensure I2C bus is closed
     */
    [Symbol.dispose]() {
        this.close();
    }
}

module.exports = {
    BNO08X_I2C,
    _BNO08X_DEFAULT_ADDRESS
};
