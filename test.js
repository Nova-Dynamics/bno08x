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
        
        // Enable accelerometer
        console.log('Enabling accelerometer...');
        await imu.enable_feature(REPORTS.GRAVITY);
        
        // Read accelerometer data for 5 seconds
        console.log('Reading accelerometer data...\n');
        
        for (let i = 0; i < 50; i++) {
            try {
                let [x, y, z] = imu.gravity;
                let pitch = Math.atan2(x, Math.sqrt(y * y + z * z)) / (Math.PI / 180)
                let roll = Math.atan2(-y, z) / (Math.PI / 180)
                
                console.log(`Pitch: ${pitch.toFixed(2)}°, Roll: ${roll.toFixed(2)}°`);
            } catch (error) {
                console.log(`Reading error: ${error.message}`);
            }
            
            await new Promise(resolve => setTimeout(resolve, 100));
        }
        
        console.log('\nExample completed!');
        
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