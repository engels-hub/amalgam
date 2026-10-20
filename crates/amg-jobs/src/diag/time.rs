//! Minimal UTC formatting for crash-report names (no date-time dependency).

use std::time::{SystemTime, UNIX_EPOCH};

/// UTC civil time with milliseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Utc {
    pub(crate) year: i64,
    pub(crate) month: u32,
    pub(crate) day: u32,
    pub(crate) hour: u32,
    pub(crate) minute: u32,
    pub(crate) second: u32,
    pub(crate) millis: u32,
}

impl Utc {
    pub(crate) fn now() -> Self {
        Self::from_system(SystemTime::now())
    }

    pub(crate) fn from_system(t: SystemTime) -> Self {
        let d = t.duration_since(UNIX_EPOCH).unwrap_or_default();
        let secs = d.as_secs() as i64;
        let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
        let (year, month, day) = civil_from_days(days);
        Self {
            year,
            month,
            day,
            hour: (rem / 3600) as u32,
            minute: (rem % 3600 / 60) as u32,
            second: (rem % 60) as u32,
            millis: d.subsec_millis(),
        }
    }

    /// `20261020T153012.123Z`: sortable and valid in Windows file names.
    pub(crate) fn compact(&self) -> String {
        format!(
            "{:04}{:02}{:02}T{:02}{:02}{:02}.{:03}Z",
            self.year, self.month, self.day, self.hour, self.minute, self.second, self.millis
        )
    }

    /// `2026-10-20T15:30:12.123Z` (RFC 3339).
    pub(crate) fn rfc3339(&self) -> String {
        format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{:03}Z",
            self.year, self.month, self.day, self.hour, self.minute, self.second, self.millis
        )
    }
}

/// Days since 1970-01-01 to (year, month, day), proleptic Gregorian (H. Hinnant's algorithm).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn known_instants_format() {
        let t = |s: u64, ms: u64| {
            Utc::from_system(UNIX_EPOCH + Duration::from_secs(s) + Duration::from_millis(ms))
        };
        assert_eq!(t(0, 0).rfc3339(), "1970-01-01T00:00:00.000Z");
        // 2000-02-29 (leap day) 12:34:56.789
        assert_eq!(t(951_827_696, 789).rfc3339(), "2000-02-29T12:34:56.789Z");
        // 2026-10-20 15:30:12.123
        let x = t(1_792_510_212, 123);
        assert_eq!(x.rfc3339(), "2026-10-20T15:30:12.123Z");
        assert_eq!(x.compact(), "20261020T153012.123Z");
        // 2100-03-01 (not a leap year)
        assert_eq!(t(4_107_542_400, 0).rfc3339(), "2100-03-01T00:00:00.000Z");
    }
}
