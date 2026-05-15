/**
 * IIO sysfs scan-element catalog for the bno086 kernel driver.
 *
 * The catalog of report ids, kinds, q-shifts, and event names lives in
 * `base.js` (REPORT_META). This file only owns the IIO-specific bits:
 *   - which sysfs scan-element bases make up each report (`iioBases` from
 *     REPORT_META, indexed here by report id and by sysfs base name)
 *   - the soft-timestamp scan-element name
 */

const path = require('path');

const {
    REPORT_META_BY_ID,
} = require('./base');

const TIMESTAMP_BASE = 'in_timestamp';

/**
 * Map: numeric SHTP report id -> {
 *     reportId, meta, sysfsBases: [string, ...]
 * }
 *
 * Only includes reports that the kernel driver actually exposes via sysfs
 * (meta.iioBases !== null).
 */
const IIO_REPORT_BY_ID = (() => {
    const map = new Map();
    for (const [idStr, meta] of Object.entries(REPORT_META_BY_ID)) {
        if (!meta.iioBases) continue;
        const reportId = Number(idStr);
        map.set(reportId, {
            reportId,
            meta,
            sysfsBases: meta.iioBases.slice(),
        });
    }
    return map;
})();

/**
 * Map: sysfs base name (e.g. 'in_accel_x') -> { reportId, meta, axisIndex }
 */
const IIO_BASE_LOOKUP = (() => {
    const map = new Map();
    for (const entry of IIO_REPORT_BY_ID.values()) {
        entry.sysfsBases.forEach((base, axisIndex) => {
            map.set(base, { reportId: entry.reportId, meta: entry.meta, axisIndex });
        });
    }
    return map;
})();

function getIioReport(reportId) {
    return IIO_REPORT_BY_ID.get(reportId) || null;
}

function lookupSysfsBase(base) {
    return IIO_BASE_LOOKUP.get(base) || null;
}

module.exports = {
    TIMESTAMP_BASE,
    IIO_REPORT_BY_ID,
    IIO_BASE_LOOKUP,
    getIioReport,
    lookupSysfsBase,
};
