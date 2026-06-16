#!/usr/bin/env node

/**
 * Linux IIO multi-report example for BNO08x.
 *
 * Run with: sudo node test_iio.js
 *
 * Mirrors the I2C example: enable features by REPORTS.* id and listen on the
 * canonical per-report event names.
 *
 * Environment:
 *   BNO_IIO_DEVICE       iio:deviceN node               (default: iio:device0)
 *   BNO_IIO_REPORTS      comma list of REPORTS keys     (default: GAME_ROTATION_VECTOR,LINEAR_ACCELERATION,GRAVITY)
 *   BNO_IIO_HZ           sampling Hz applied to all enabled reports (default: 50)
 *   BNO_IIO_POLL_MS      userspace poll interval in ms  (default: derived from Hz)
 *   BNO_IIO_DURATION_S   capture duration in seconds    (default: 6)
 *   BNO_IIO_DEBUG        set to 1 for backend debug logs
 *   BNO_IIO_DEBUG_RAW    set to 1 to dump record hex
 */

const { BNO08X_IIO, REPORTS, REPORT_META } = require('./index');

function parseReportList(raw) {
    return String(raw)
        .split(',')
        .map((s) => s.trim())
        .filter(Boolean)
        .map((s) => {
            const id = REPORTS[s];
            if (id === undefined) {
                throw new Error(`Unknown REPORTS key: ${s}. Valid: ${Object.keys(REPORTS).join(',')}`);
            }
            return id;
        });
}

function fmt(n, width = 7, digits = 3) {
    if (!Number.isFinite(n)) return 'nan'.padStart(width);
    return n.toFixed(digits).padStart(width);
}

function shortTs(timestampNs) {
    if (typeof timestampNs !== 'bigint') return 'n/a';
    return (timestampNs % 1000000n).toString().padStart(6, '0');
}

async function main() {
    const iioDevice = process.env.BNO_IIO_DEVICE || 'iio:device0';
    const reportIds = parseReportList(process.env.BNO_IIO_REPORTS || 'GAME_ROTATION_VECTOR,LINEAR_ACCELERATION,GRAVITY');
    const hz = Number(process.env.BNO_IIO_HZ || 50);
    const pollIntervalMs = process.env.BNO_IIO_POLL_MS !== undefined
        ? Number(process.env.BNO_IIO_POLL_MS)
        : Math.max(1, Math.round(1000 / hz));
    const durationS = Number(process.env.BNO_IIO_DURATION_S || 6);
    const debug = process.env.BNO_IIO_DEBUG === '1';
    const debugRaw = process.env.BNO_IIO_DEBUG_RAW === '1';
    const reportIntervalUs = Math.max(1, Math.round(1e6 / hz));

    console.log('BNO08x IIO multi-report example');
    console.log('================================');
    console.log(`Device:   ${iioDevice}`);
    console.log(`Reports:  ${reportIds.map((id) => REPORT_META[id].key).join(', ')}`);
    console.log(`Rate:     ${hz} Hz   poll: ${pollIntervalMs} ms   duration: ${durationS}s`);

    const imu = new BNO08X_IIO(iioDevice, {
        pollIntervalMs,
        debug,
        debugRaw,
    });

    try {
        const discovered = imu.getReports();
        const missing = reportIds.filter((id) => !discovered[id]);
        if (missing.length > 0) {
            const names = missing.map((id) => REPORT_META[id]?.key || `0x${id.toString(16)}`);
            const available = Object.values(discovered).map((r) => r.key).join(',');
            throw new Error(`Reports not exposed by driver: ${names.join(',')}. Available: ${available}`);
        }

        // Clean up any leftover state from a prior run.
        for (const idStr of Object.keys(discovered)) {
            try { await imu.disable_feature(Number(idStr)); } catch (_) { /* ignore */ }
        }

        for (const id of reportIds) {
            await imu.enable_feature(id, reportIntervalUs);
        }

        const counts = new Map(reportIds.map((id) => [id, 0]));
        imu.on('report', (report) => {
            counts.set(report.reportId, (counts.get(report.reportId) || 0) + 1);
            const ts = shortTs(report.timestampNs);
            if (report.kind === 'quat') {
                const [qi, qj, qk, qr] = report.data;
                const flag = report.normLikelyValid ? '' : ` !n=${report.norm.toFixed(3)}`;
                console.log(`${report.reportKey.padEnd(22)} q=[${fmt(qi)},${fmt(qj)},${fmt(qk)},${fmt(qr)}] t=${ts}${flag}`);
            } else if (report.kind === 'vec3') {
                const [x, y, z] = report.data;
                console.log(`${report.reportKey.padEnd(22)} xyz=[${fmt(x)},${fmt(y)},${fmt(z)}] t=${ts}`);
            }
        });

        let stoppedOnDisconnect = false;
        imu.on('error', (err) => {
            const message = String(err && err.message ? err.message : err);
            const isDisconnect = /(ENODEV|ENXIO|EIO|EBADF|disconnected|not connected|No such device)/i.test(message);
            if (isDisconnect) {
                if (stoppedOnDisconnect) return;
                stoppedOnDisconnect = true;
                console.error(`Device disconnected, stopping: ${message}`);
                imu.close();
                return;
            }
            console.error(`IIO error: ${message}`);
        });

        await new Promise((resolve) => setTimeout(resolve, durationS * 1000));

        console.log('\nRecord counts:');
        for (const [id, n] of counts) {
            console.log(`  ${REPORT_META[id].key}: ${n}`);
        }
    } finally {
        imu.close();
    }
}

if (require.main === module) {
    main().catch((err) => { console.error(err); process.exit(1); });
}
