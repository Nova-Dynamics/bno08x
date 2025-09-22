# BNO08x Node.js Library

A Node.js port of the [Adafruit BNO08x CircuitPython library](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x) for interfacing with the Hillcrest Laboratories BNO08x IMU (Inertial Measurement Unit).

## Features

- **Full sensor suite support**: Accelerometer, gyroscope, magnetometer, and more
- **Advanced motion detection**: Quaternion/rotation vectors, linear acceleration, gravity vector
- **Smart features**: Step counting, shake detection, activity classification, stability analysis
- **Calibration support**: Built-in sensor calibration and calibration data saving
- **I2C communication**: Uses the `i2c-bus` library for reliable I2C communication

## Installation

1. Make sure I2C is enabled on your system (for Raspberry Pi):
```bash
sudo raspi-config
# Navigate to Interface Options -> I2C -> Enable
```

2. Install this library:
```bash
# Install from npm:
npm install @novadynamics/bno08x

# Or clone/copy this repository and install dependencies:
npm install
```

## Quick Start

```javascript
const { BNO08X_I2C, REPORTS } = require('bno08x');

async function main() {
    // Initialize IMU on I2C bus 1, address 0x4B
    const imu = new BNO08X_I2C(1, { address: 0x4B });
    
    // Wait for initialization
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    // Enable accelerometer
    await imu.enable_feature(REPORTS.ACCELEROMETER);
    
    // Read data
    setInterval(() => {
        try {
            const acceleration = imu.acceleration;
            console.log(`Accel: ${acceleration.map(v => v.toFixed(3)).join(', ')} m/s²`);
        } catch (error) {
            console.error(`Read error: ${error.message}`);
        }
    }, 100);
}

main().catch(console.error);
```

## API Reference

### BNO08X_I2C Class

#### Constructor
```javascript
new BNO08X_I2C(bus_number, options)
```

**Parameters:**
- `bus_number` (number): I2C bus number (e.g., 1 for `/dev/i2c-1`)
- `options` (object):
  - `address` (number): I2C address (default: 0x4A)
  - `debug` (boolean): Enable debug output (default: false)
  - `reset` (object): Reset pin control (not implemented)

#### Methods

##### `async enable_feature(feature_id)`
Enable a specific sensor feature.

**Parameters:**
- `feature_id` (number): Feature to enable (use `REPORTS` constants)

##### `begin_calibration()`
Start the sensor's self-calibration routine.

##### `get calibration_status`
Get the current calibration status (0-3, where 3 is fully calibrated).

##### `async save_calibration_data()`
Save calibration data to the sensor's non-volatile memory.

##### `close()`
Close the I2C connection.

#### Properties (Sensor Readings)

All properties automatically process available packets and return the latest reading.

##### Basic Sensors
- `acceleration` - Calibrated acceleration (m/s²) [x, y, z]
- `gyro` - Calibrated gyroscope (rad/s) [x, y, z]
- `magnetic` - Calibrated magnetometer (µTesla) [x, y, z]

##### Motion Vectors
- `quaternion` - Rotation vector as quaternion [i, j, k, real]
- `game_quaternion` - Game rotation vector (no magnetic correction)
- `geomagnetic_quaternion` - Geomagnetic rotation vector
- `linear_acceleration` - Linear acceleration (gravity removed)
- `gravity` - Gravity vector [x, y, z]

##### Activity Detection
- `steps` - Step count since initialization
- `shake` - Boolean, true if shake detected (auto-clears on read)
- `stability_classification` - Current stability assessment
- `activity_classification` - Current activity classification

##### Raw Sensors
- `raw_acceleration` - Raw accelerometer values
- `raw_gyro` - Raw gyroscope values  
- `raw_magnetic` - Raw magnetometer values

### Constants

#### Report IDs
```javascript
const { REPORTS } = require('@novadynamics/bno08x');

REPORTS.ACCELEROMETER           // Calibrated accelerometer
REPORTS.GYROSCOPE              // Calibrated gyroscope
REPORTS.MAGNETOMETER           // Calibrated magnetometer
REPORTS.LINEAR_ACCELERATION    // Linear acceleration
REPORTS.ROTATION_VECTOR        // Quaternion
REPORTS.GRAVITY                // Gravity vector
REPORTS.GAME_ROTATION_VECTOR   // Game quaternion
REPORTS.GEOMAGNETIC_ROTATION_VECTOR // Geomagnetic quaternion
REPORTS.STEP_COUNTER           // Step counter
REPORTS.SHAKE_DETECTOR         // Shake detection
REPORTS.STABILITY_CLASSIFIER   // Stability classification
REPORTS.ACTIVITY_CLASSIFIER    // Activity classification
REPORTS.RAW_ACCELEROMETER      // Raw accelerometer
REPORTS.RAW_GYROSCOPE          // Raw gyroscope
REPORTS.RAW_MAGNETOMETER       // Raw magnetometer
```

#### Other Constants
```javascript
const { BNO08X_DEFAULT_ADDRESS, ACCURACY_STATUS } = require('@novadynamics/bno08x');

BNO08X_DEFAULT_ADDRESS  // 0x4A - Default I2C address
ACCURACY_STATUS         // Array of accuracy descriptions
```

## Examples

### Basic Accelerometer Reading
```javascript
const { BNO08X_I2C, REPORTS } = require('@novadynamics/bno08x');

async function read_accelerometer() {
    const imu = new BNO08X_I2C(1, { address: 0x4B });
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    await imu.enable_feature(REPORTS.ACCELEROMETER);
    
    setInterval(() => {
        const accel = imu.acceleration;
        console.log(`Accel: ${accel.map(v => v.toFixed(3)).join(', ')} m/s²`);
    }, 100);
}
```

### Full Sensor Suite
```javascript
async function read_all_sensors() {
    const imu = new BNO08X_I2C(1, { address: 0x4B });
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    // Enable multiple sensors
    await imu.enable_feature(REPORTS.ACCELEROMETER);
    await imu.enable_feature(REPORTS.GYROSCOPE);
    await imu.enable_feature(REPORTS.MAGNETOMETER);
    await imu.enable_feature(REPORTS.ROTATION_VECTOR);
    
    setInterval(() => {
        console.log('Acceleration:', imu.acceleration);
        console.log('Gyroscope:', imu.gyro);
        console.log('Magnetometer:', imu.magnetic);
        console.log('Quaternion:', imu.quaternion);
        console.log('---');
    }, 200);
}
```

### Activity and Motion Detection
```javascript
async function detect_motion() {
    const imu = new BNO08X_I2C(1, { address: 0x4B });
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    await imu.enable_feature(REPORTS.STEP_COUNTER);
    await imu.enable_feature(REPORTS.SHAKE_DETECTOR);
    await imu.enable_feature(REPORTS.ACTIVITY_CLASSIFIER);
    
    setInterval(() => {
        console.log('Steps:', imu.steps);
        console.log('Activity:', imu.activity_classification?.most_likely);
        
        if (imu.shake) {
            console.log('🎉 SHAKE DETECTED!');
        }
    }, 500);
}
```

### Calibration
```javascript
async function calibrate_sensor() {
    const imu = new BNO08X_I2C(1, { address: 0x4B });
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    await imu.enable_feature(REPORTS.MAGNETOMETER);
    
    console.log('Starting calibration...');
    imu.begin_calibration();
    
    // Monitor calibration progress
    const monitor = setInterval(() => {
        const status = imu.calibration_status;
        console.log(`Calibration status: ${status}/3`);
        
        if (status >= 3) {
            console.log('Calibration complete!');
            imu.save_calibration_data();
            clearInterval(monitor);
        }
    }, 1000);
}
```

## Troubleshooting

### Permission Errors
If you get permission errors accessing I2C:
```bash
sudo node your_script.js
# or add your user to the i2c group:
sudo usermod -a -G i2c $USER
```

### I2C Not Found
Make sure I2C is enabled:
```bash
# Check if I2C devices are visible
sudo i2cdetect -y 1

# Enable I2C on Raspberry Pi
sudo raspi-config
```

### Device Not Responding
- Check wiring connections
- Verify I2C address with `sudo i2cdetect -y 1`
- Try different I2C addresses (0x4A, 0x4B)
- Check power supply (3.3V)

### Common Issues
1. **"No packet available"** - Normal during initialization
2. **"Feature not enabled"** - Call `enable_feature()` first
3. **Timeout errors** - Check I2C connections and address

## Hardware Notes

- **Power**: 3.3V (do not use 5V)
- **I2C**: Standard I2C communication
- **Default Address**: 0x4A (some modules use 0x4B)
- **Reset**: Optional reset pin (not implemented in this version)

## License

This is a Node.js port of the [Adafruit BNO08x CircuitPython library](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x).

Original CircuitPython code by Bryan Siepert for Adafruit Industries.  
JavaScript port by Sean Sullivan (via vibe coding :P).

SPDX-License-Identifier: MIT