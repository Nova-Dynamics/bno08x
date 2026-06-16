#!/usr/bin/env node

/**
 * I2C example for BNO08x Node.js library.
 *
 * Run with: sudo node test_i2c.js
 */

const { BNO08X_I2C, REPORTS } = require('./index');

async function simple_example_i2c() {
    console.log('BNO08x I2C Example');
    console.log('==================');

    let imu = null;

    try {
        const bus = Number(process.env.BNO_I2C_BUS || 12);
        const address = Number(process.env.BNO_I2C_ADDR || 0x4A);

        imu = new BNO08X_I2C(bus, {
            address,
            debug: true,
        });

        console.log('IMU initialized, waiting for startup...');
        await new Promise(resolve => setTimeout(resolve, 2000));

        let last = Date.now();
        imu.on('game_quaternion', () => {
            const cur = Date.now();
            console.log(`Game Quaternion (${cur - last} ms since last)`);
            last = cur;
        });

        console.log('Enabling game rotation...');
        await imu.enable_feature(REPORTS.GAME_ROTATION_VECTOR, 8333);
        imu.start_auto_reporting(1000);
        await new Promise(resolve => setTimeout(resolve, 6000));
    } catch (error) {
        console.error(`Error: ${error.message}`);
        if (error.message.includes('EACCES')) {
            console.error('Try running with sudo or add your user to the i2c group.');
        }
    } finally {
        if (imu) {
            imu.close();
        }
    }
}

if (require.main === module) {
    simple_example_i2c().catch(console.error);
}
