//! UTC timestamps for dump names and manifests, without a date-time dependency.

use std::time::{SystemTime, UNIX_EPOCH};

/// `(year, month, day)` of the proleptic Gregorian calendar for days since 1970-01-01
/// (H. Hinnant's `civil_from_days`).
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "day in 1..=31 and month in 1..=12"
    )]
    (year, month as u32, day as u32)
}

/// Seconds since the Unix epoch (0 if the clock is before it).
fn now_secs() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX))
}

fn parts(secs: i64) -> (i64, u32, u32, i64, i64, i64) {
    let days = secs.div_euclid(86_400);
    let rem = secs.rem_euclid(86_400);
    let (y, m, d) = civil_from_days(days);
    (y, m, d, rem / 3600, rem % 3600 / 60, rem % 60)
}

/// `20261020T140311Z`, for directory names.
pub(crate) fn compact(secs: i64) -> String {
    let (y, m, d, hh, mm, ss) = parts(secs);
    format!("{y:04}{m:02}{d:02}T{hh:02}{mm:02}{ss:02}Z")
}

/// `2026-10-20T14:03:11Z` (RFC 3339), for manifests.
pub(crate) fn rfc3339(secs: i64) -> String {
    let (y, m, d, hh, mm, ss) = parts(secs);
    format!("{y:04}-{m:02}-{d:02}T{hh:02}:{mm:02}:{ss:02}Z")
}

/// The current time in seconds since the epoch.
pub(crate) fn now() -> i64 {
    now_secs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_known_instants() {
        assert_eq!(compact(0), "19700101T000000Z");
        assert_eq!(rfc3339(0), "1970-01-01T00:00:00Z");
        // 2026-10-20T14:03:11Z.
        assert_eq!(rfc3339(1_792_504_991), "2026-10-20T14:03:11Z");
        assert_eq!(compact(1_792_504_991), "20261020T140311Z");
        // Leap day.
        assert_eq!(rfc3339(951_782_400), "2000-02-29T00:00:00Z");
    }
}
