# BNO08x Node.js Library

A Node.js port of the [Adafruit BNO08x CircuitPython library](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x) for interfacing with the Hillcrest Laboratories BNO08x IMU (Inertial Measurement Unit).

## AI-Generated Content Disclosure

Large parts of this project were developed with AI assistance. Organizations or projects that
forbid AI-generated code should treat this repository as out of policy.

## Features

- **Full sensor suite support**: Accelerometer, gyroscope, magnetometer, and more
- **Advanced motion detection**: Quaternion/rotation vectors, linear acceleration, gravity vector
- **Smart features**: Step counting, shake detection, activity classification, stability analysis
- **Calibration support**: Built-in sensor calibration and calibration data saving
- **I2C communication**: Uses the `i2c-bus` library for reliable I2C communication
- **Linux IIO backend**: Read quaternion data from `/dev/iio:deviceN` exposed by the bno086 kernel driver

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

### I2C Backend (existing)

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

### Linux IIO Backend

The IIO backend exposes the same API surface as the I2C backend: enable
features by their numeric `REPORTS.*` id and listen on the canonical per-report
events (`accelerometer`, `gyroscope`, `quaternion`, `game_quaternion`,
`linear_acceleration`, `gravity`, ...).

```javascript
const { BNO08X_IIO, REPORTS } = require('@novadynamics/bno08x');

async function main() {
    const imu = new BNO08X_IIO('iio:device0');

    imu.on('game_quaternion', (quat, accuracy) => {
        console.log('Quaternion:', quat.map(v => v.toFixed(5)).join(', '));
    });

    // 100 Hz = 10000 µs report interval. enable_feature() auto-starts the
    // buffer and poll loop on the first call.
    await imu.enable_feature(REPORTS.GAME_ROTATION_VECTOR, 10000);
    await imu.enable_feature(REPORTS.LINEAR_ACCELERATION, 10000);

    await new Promise(resolve => setTimeout(resolve, 5000));

    // Cached getters work the same as on I2C.
    console.log('latest gravity:', imu.gravity);

    imu.close();
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

### BNO08X_IIO Class

IIO backend for Linux systems where the bno086 kernel driver exposes an IIO
device. Public API mirrors `BNO08X_I2C`.

#### Constructor
```javascript
new BNO08X_IIO(device, options)
```

**Parameters:**
- `device` (number|string): IIO device index, `iio:deviceN`, or `/dev/iio:deviceN`
- `options` (object):
    - `pollIntervalMs` (number): Poll interval for reads (default: 20)
    - `bufferLength` (number|null): Writes `buffer/length` before enabling (default: leave kernel default)
    - `sysfsRoot` (string): Sysfs root path (default: `/sys/bus/iio/devices`)
    - `debug` (boolean): Enable debug output (default: false)
    - `debugRaw` (boolean): Dump raw record hex (default: false)
    - `quaternionNormTolerance` (number): Tolerance for the quaternion norm validity flag (default: 0.25)

#### Methods

- `async enable_feature(reportId, reportIntervalUs)` — Enable an SH-2 report.
  `reportId` is a numeric `REPORTS.*` id. `reportIntervalUs` (microseconds)
  is converted to a sysfs sampling frequency in Hz. The buffer and poll loop
  start automatically on the first enable; subsequent calls hot-restart the
  buffer to add the new feature.
- `async disable_feature(reportId)` — Symmetric inverse. Stops the buffer/poll
  loop when the last feature is disabled.
- `setReorientationQuaternion({x, y, z, w})` — Apply a sensor reorientation
  (tare) by writing the kernel's `reorientation_quaternion` sysfs attribute.
  Pass `{x:0, y:0, z:0, w:1}` to clear the tare. The helper converts the
  unit-quaternion components to Q14 signed 16-bit integers
  (`clamp(round(v * 16384), -32768, 32767)`) before writing; the kernel
  driver itself never touches floating point. The tare is volatile and is
  not persisted to the chip's flash.

  > **Reorientation convention.** This part's firmware interprets the SH-2
  > SET_REORIENTATION quaternion in an axis frame whose X and Y are swapped
  > relative to the gravity-report frame. The five `frs_*_orientation` FRS
  > records are unprogrammed (firmware default = identity), so the swap is a
  > firmware ABI quirk, not an FRS misconfig. If you compute a tare yourself
  > using gravity reports, swap `qx` and `qy` before passing the quaternion
  > here. The high-level helper below already does this for you.
- `async tareGravityToZ({samples, timeoutMs, intervalUs} = {})` — Convenience
  helper that tares the sensor so the current direction of gravity becomes
  `+Z` (Z-up convention). Averages `samples` (default 20) gravity reports,
  computes the shortest-arc rotation that maps measured gravity onto `+Z`,
  applies the X/Y swap quirk above, and writes the resulting quaternion via
  `setReorientationQuaternion`. If `REPORTS.GRAVITY` is not already enabled
  it is enabled at `intervalUs` (default 20 ms) for the duration of the
  tare and then disabled again. Returns
  `{ quaternion: {x,y,z,w}, gravityBefore: [x,y,z] }`. Throws if the
  required samples cannot be collected within `timeoutMs` (default 2000 ms).
- `getReports()` — Diagnostic. Returns discovered reports keyed by SHTP id.
- `getEnabledReports()` — Diagnostic. Returns ids of currently enabled reports.
- `close()` — Stop the buffer/poll loop and close the device file descriptor.

#### Properties

The same cached getters as `BNO08X_I2C`: `acceleration`, `gyro`, `magnetic`,
`linear_acceleration`, `quaternion`, `game_quaternion`,
`geomagnetic_quaternion`, `gravity`, `raw_acceleration`, `raw_gyro`,
`raw_magnetic`. Each returns the most recent decoded value populated by the
poll loop.

#### Events

Per-report events match the I2C backend names (`accelerometer`, `gyroscope`,
`magnetometer`, `linear_acceleration`, `quaternion`, `game_quaternion`,
`geomagnetic_quaternion`, `gravity`, `raw_accelerometer`, `raw_gyroscope`,
`raw_magnetometer`). The generic `'report'` event fires for every record with
a structured payload that includes `reportId`, `reportKey`, `kind`, `data`,
`timestampNs`, `raw`, `qShift`, and (for quaternions) `norm` /
`normLikelyValid`. An `'error'` event is emitted on read failures.

#### Limitations

- Only the SH-2 reports the kernel driver exposes via IIO scan elements can
  be enabled. Today this is: accelerometer, gyroscope, magnetometer, linear
  acceleration, gravity, rotation vector, game rotation vector, geomagnetic
  rotation vector, and the three raw sensor channels.
- Calibration commands (`begin_calibration`, `save_calibration_data`) and
  `start_auto_reporting` are I2C-only.

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
If you get permission errors accessing I2C or IIO:
```bash
sudo node your_script.js
# or add your user to the i2c group (I2C only):
sudo usermod -a -G i2c $USER
```

For IIO, ensure your udev rules allow access to `/dev/iio:deviceN` and sysfs writes under `/sys/bus/iio/devices/iio:deviceN/`.

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

- Original CircuitPython code by Bryan Siepert for Adafruit Industries.
- JavaScript port by Sean Sullivan (via vibe coding :P).
- IIO support added by Wilkins White

SPDX-License-Identifier: MIT
