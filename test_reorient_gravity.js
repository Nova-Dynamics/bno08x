#!/usr/bin/env node
/*
 * Reorientation example (Z-up convention).
 *
 *   1. Read the gravity vector from the BNO086.
 *   2. Tare so the current direction of gravity becomes +Z, using the
 *      `tareGravityToZ` convenience helper which encapsulates both the
 *      Z-up math and the BNO086 firmware's X/Y axis-swap quirk.
 *   3. Re-read the gravity vector and print it. Should be ~[0, 0, +g].
 *
 * Run with: sudo node test_reorient_gravity.js
 *
 * Env vars:
 *   BNO_IIO_DEVICE   iio:deviceN node           (default: iio:device0)
 *   BNO_IIO_HZ       sample rate                (default: 50)
 *   BNO_SAMPLES      gravity samples to average (default: 25)
 *   BNO_SETTLE_MS    settle after applying tare (default: 1500)
 */

'use strict';

const { BNO08X_IIO, REPORTS } = require('./index');

const DEVICE     = process.env.BNO_IIO_DEVICE  || 'iio:device0';
const HZ         = parseInt(process.env.BNO_IIO_HZ    || '50',   10);
const SAMPLES    = parseInt(process.env.BNO_SAMPLES   || '25',   10);
const SETTLE_MS  = parseInt(process.env.BNO_SETTLE_MS || '1500', 10);

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
function vlen(v)   { return Math.hypot(v[0], v[1], v[2]); }
function fmt(v)    { return '[' + v.map(x => x.toFixed(3).padStart(7)).join(', ') + ']'; }

function collectGravity(imu, n) {
    return new Promise((resolve, reject) => {
        const acc = [0, 0, 0];
        let got = 0;
        const cleanup = () => { imu.off('report', onReport); imu.off('error', onError); };
        const onReport = (report) => {
            if (report.reportId !== REPORTS.GRAVITY || report.kind !== 'vec3') return;
            acc[0] += report.data[0]; acc[1] += report.data[1]; acc[2] += report.data[2];
            got++;
            if (got >= n) { cleanup(); resolve([acc[0]/got, acc[1]/got, acc[2]/got]); }
        };
        const onError = (err) => { cleanup(); reject(err); };
        imu.on('report', onReport);
        imu.on('error',  onError);
    });
}

async function main() {
    const imu = new BNO08X_IIO(DEVICE, { pollIntervalMs: Math.max(1, Math.round(1000 / HZ)) });
    const intervalUs = Math.max(1, Math.round(1e6 / HZ));

    try {
        // Clear any prior tare. All-zero quaternion is the SH-2 "clear" form.
        imu.setReorientationQuaternion({ x: 0, y: 0, z: 0, w: 0 });

        await imu.enable_feature(REPORTS.GRAVITY, intervalUs);
        await sleep(SETTLE_MS);

        const before = await collectGravity(imu, SAMPLES);
        console.log(`gravity (before): ${fmt(before)}  |g| = ${vlen(before).toFixed(3)}`);

        const { quaternion } = await imu.tareGravityToZ({ samples: SAMPLES });
        console.log(`tare quaternion:  [x=${quaternion.x.toFixed(4)}, y=${quaternion.y.toFixed(4)}, z=${quaternion.z.toFixed(4)}, w=${quaternion.w.toFixed(4)}]`);

        await sleep(SETTLE_MS);
        const after = await collectGravity(imu, SAMPLES);
        console.log(`gravity (after):  ${fmt(after)}  |g| = ${vlen(after).toFixed(3)}`);
        console.log(`expected:         [  0.000,   0.000,  +${vlen(before).toFixed(3)}]`);
    } finally {
        try { imu.setReorientationQuaternion({ x: 0, y: 0, z: 0, w: 0 }); } catch (_) {}
        try { await imu.disable_feature(REPORTS.GRAVITY); } catch (_) {}
        imu.close();
    }
}

main().catch(err => {
    console.error('FAIL:', err && err.stack || err);
    process.exit(1);
});
