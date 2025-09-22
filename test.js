#!/usr/bin/env node

/**
 * Simple example for BNO08x Node.js library
 * 
 * Basic usage example showing how to read accelerometer data
 * Run with: sudo node test.js
 */

const { BNO08X_I2C, REPORTS } = require('./index');

async function simple_example() {
    console.log('BNO08x Simple Example');
    console.log('=====================');
    
    let imu = null;
    
    try {
        // Initialize the IMU
        imu = new BNO08X_I2C(1, {
            address: 0x4B,
            debug: false  // Set to true for verbose output
        });
        
        console.log('IMU initialized, waiting for startup...');
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        let last = Date.now();

        imu.on("game_quaternion", (data) => {
            let cur = Date.now();
            console.log(`Game Quaternion (${(cur - last)} ms since last):`);
            last = cur;
            // console.log(data);
        });

        // Enable accelerometer
        console.log('Enabling game rotation...');
        await imu.enable_feature(REPORTS.GAME_ROTATION_VECTOR, 8333);

        imu.start_auto_reporting(1000);
        await new Promise(resolve => setTimeout(resolve, 6000));
 
        
    } catch (error) {
        console.error(`Error: ${error.message}`);
        
        if (error.message.includes('EACCES')) {
            console.error('💡 Try running with sudo for I2C access');
        }
        
    } finally {
        if (imu) {
            imu.close();
        }
    }
}

// Run the example
if (require.main === module) {
    simple_example().catch(console.error);
}