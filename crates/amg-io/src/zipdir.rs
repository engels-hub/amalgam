//! Minimal zip central-directory reader: entry names only, enough for [`probe`](crate::probe)
//! to tell a 3MF from an `.amg` project or an office document. The full reader (data, ZIP64
//! sizes, data descriptors, bomb limits) is IO-03's.

use crate::endpoint::Source;
use crate::error::IoError;
use crate::options::ReadLimits;

const EOCD_SIG: u32 = 0x0605_4b50;
const EOCD64_LOCATOR_SIG: u32 = 0x0706_4b50;
const EOCD64_SIG: u32 = 0x0606_4b50;
const CDFH_SIG: u32 = 0x0201_4b50;
/// End-of-central-directory record: 22 bytes plus a comment of at most 64 KiB.
const EOCD_SEARCH: u64 = 22 + 65_535;
/// A central directory beyond this is not worth reading for a name list.
const MAX_CD_BYTES: u64 = 512 << 20;

fn u16le(b: &[u8], o: usize) -> u64 {
    u64::from(u16::from_le_bytes([b[o], b[o + 1]]))
}
fn u32le(b: &[u8], o: usize) -> u64 {
    u64::from(u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]))
}
fn u64le(b: &[u8], o: usize) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[o..o + 8]);
    u64::from_le_bytes(a)
}

/// Entry names of the zip in `src`, or `None` if it is not a readable zip.
///
/// # Errors
/// [`IoError::Read`] for I/O failures and [`IoError::LimitExceeded`] when the entry count is
/// above `limits.max_zip_entries`.
pub(crate) fn entry_names(
    src: &Source<'_>,
    limits: &ReadLimits,
) -> Result<Option<Vec<String>>, IoError> {
    let len = src.len()?;
    if len < 22 {
        return Ok(None);
    }
    let tail_len = len.min(EOCD_SEARCH + 20);
    let tail_start = len - tail_len;
    let mut tail = vec![0u8; tail_len as usize];
    src.read_exact_at(tail_start, &mut tail)?;
    let Some(eocd) = (0..=tail.len() - 22)
        .rev()
        .find(|&i| u32le(&tail, i) == u64::from(EOCD_SIG))
    else {
        return Ok(None);
    };
    let mut entries = u16le(&tail, eocd + 10);
    let mut cd_size = u32le(&tail, eocd + 12);
    let mut cd_off = u32le(&tail, eocd + 16);
    if (entries == 0xFFFF || cd_size == 0xFFFF_FFFF || cd_off == 0xFFFF_FFFF)
        && eocd >= 20
        && u32le(&tail, eocd - 20) == u64::from(EOCD64_LOCATOR_SIG)
    {
        let off64 = u64le(&tail, eocd - 20 + 8);
        let mut rec = [0u8; 56];
        if off64.checked_add(56).is_none_or(|e| e > len) {
            return Ok(None);
        }
        src.read_exact_at(off64, &mut rec)?;
        if u32le(&rec, 0) != u64::from(EOCD64_SIG) {
            return Ok(None);
        }
        entries = u64le(&rec, 32);
        cd_size = u64le(&rec, 40);
        cd_off = u64le(&rec, 48);
    }
    if entries > u64::from(limits.max_zip_entries) {
        return Err(IoError::LimitExceeded {
            what: "zip entries",
            limit: u64::from(limits.max_zip_entries),
            actual: entries,
        });
    }
    // The allocation rule: never more than the file can hold.
    if cd_off.checked_add(cd_size).is_none_or(|e| e > len) || cd_size > MAX_CD_BYTES {
        return Ok(None);
    }
    let mut cd = vec![0u8; cd_size as usize];
    src.read_exact_at(cd_off, &mut cd)?;
    let mut names = Vec::with_capacity(entries.min(4096) as usize);
    let mut p = 0usize;
    while p + 46 <= cd.len() && names.len() < entries as usize {
        if u32le(&cd, p) != u64::from(CDFH_SIG) {
            return Ok(None);
        }
        let n = u16le(&cd, p + 28) as usize;
        let extra = u16le(&cd, p + 30) as usize;
        let comment = u16le(&cd, p + 32) as usize;
        let Some(name) = cd.get(p + 46..p + 46 + n) else {
            return Ok(None);
        };
        names.push(String::from_utf8_lossy(name).into_owned());
        p += 46 + n + extra + comment;
    }
    Ok((names.len() as u64 == entries).then_some(names))
}

/// What a zip container is.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum ZipKind {
    ThreeMf,
    /// Amalgam's own project container (`design.json` + `manifest.json`).
    AmgProject,
    Other,
}

pub(crate) fn classify(names: &[String]) -> ZipKind {
    let has = |n: &str| names.iter().any(|e| e.eq_ignore_ascii_case(n));
    if has("design.json") && has("manifest.json") {
        return ZipKind::AmgProject;
    }
    let has_model = names.iter().any(|e| {
        let l = e.to_ascii_lowercase();
        l.starts_with("3d/") && !l.ends_with('/')
    });
    if has("[Content_Types].xml") && has_model {
        ZipKind::ThreeMf
    } else {
        ZipKind::Other
    }
}

/// Slicer-specific parts that mark a 3MF as a slicer project.
pub(crate) fn has_slicer_parts(names: &[String]) -> bool {
    names.iter().any(|e| {
        let l = e.to_ascii_lowercase();
        l.starts_with("metadata/slic3r")
            || l == "metadata/model_settings.config"
            || l == "metadata/project_settings.config"
            || l.starts_with("metadata/plate_")
    })
}

/// Test helper: a zip with stored (uncompressed) entries.
#[cfg(test)]
pub(crate) fn build_stored_zip(entries: &[(&str, &[u8])]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut cd = Vec::new();
    for (name, data) in entries {
        let off = out.len() as u32;
        let crc = 0u32; // names-only reader; the CRC is never checked
        let size = data.len() as u32;
        out.extend_from_slice(&0x0403_4b50u32.to_le_bytes());
        out.extend_from_slice(&[20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        out.extend_from_slice(&crc.to_le_bytes());
        out.extend_from_slice(&size.to_le_bytes());
        out.extend_from_slice(&size.to_le_bytes());
        out.extend_from_slice(&(name.len() as u16).to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(name.as_bytes());
        out.extend_from_slice(data);

        cd.extend_from_slice(&CDFH_SIG.to_le_bytes());
        cd.extend_from_slice(&[20, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        cd.extend_from_slice(&crc.to_le_bytes());
        cd.extend_from_slice(&size.to_le_bytes());
        cd.extend_from_slice(&size.to_le_bytes());
        cd.extend_from_slice(&(name.len() as u16).to_le_bytes());
        cd.extend_from_slice(&[0; 12]);
        cd.extend_from_slice(&off.to_le_bytes());
        cd.extend_from_slice(name.as_bytes());
    }
    let cd_off = out.len() as u32;
    out.extend_from_slice(&cd);
    out.extend_from_slice(&EOCD_SIG.to_le_bytes());
    out.extend_from_slice(&[0, 0, 0, 0]);
    out.extend_from_slice(&(entries.len() as u16).to_le_bytes());
    out.extend_from_slice(&(entries.len() as u16).to_le_bytes());
    out.extend_from_slice(&(cd.len() as u32).to_le_bytes());
    out.extend_from_slice(&cd_off.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names_of(bytes: &[u8]) -> Option<Vec<String>> {
        entry_names(&Source::Bytes(bytes), &ReadLimits::default()).expect("no io error")
    }

    #[test]
    fn lists_entry_names_of_a_stored_zip() {
        let z = build_stored_zip(&[("[Content_Types].xml", b"<x/>"), ("3D/3dmodel.model", b"m")]);
        assert_eq!(
            names_of(&z),
            Some(vec![
                "[Content_Types].xml".into(),
                "3D/3dmodel.model".into()
            ])
        );
    }

    #[test]
    fn classification() {
        let n = |v: &[&str]| v.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>();
        assert_eq!(
            classify(&n(&[
                "[Content_Types].xml",
                "_rels/.rels",
                "3D/3dmodel.model"
            ])),
            ZipKind::ThreeMf
        );
        assert_eq!(
            classify(&n(&["manifest.json", "design.json", "bodies/a.bin"])),
            ZipKind::AmgProject
        );
        // An office document is OPC too, but has no 3D part.
        assert_eq!(
            classify(&n(&["[Content_Types].xml", "word/document.xml"])),
            ZipKind::Other
        );
        assert_eq!(classify(&n(&["3D/"])), ZipKind::Other);
        assert!(has_slicer_parts(&n(&["Metadata/model_settings.config"])));
        assert!(!has_slicer_parts(&n(&["3D/3dmodel.model"])));
    }

    #[test]
    fn rejects_garbage_and_truncation_without_panicking() {
        assert_eq!(names_of(b"PK\x03\x04 not really a zip"), None);
        assert_eq!(names_of(&[]), None);
        let z = build_stored_zip(&[("a", b"1"), ("b", b"2")]);
        for cut in 0..z.len() {
            let _ = names_of(&z[..cut]); // any prefix must be handled
        }
        // Flip bytes in the central directory.
        for i in 0..z.len() {
            let mut bad = z.clone();
            bad[i] ^= 0xFF;
            let _ = names_of(&bad);
        }
    }

    #[test]
    fn entry_count_limit() {
        let z = build_stored_zip(&[("a", b""), ("b", b""), ("c", b"")]);
        let limits = ReadLimits {
            max_zip_entries: 2,
            ..ReadLimits::default()
        };
        assert!(matches!(
            entry_names(&Source::Bytes(&z), &limits),
            Err(IoError::LimitExceeded {
                what: "zip entries",
                limit: 2,
                actual: 3
            })
        ));
    }

    #[test]
    fn a_zip_comment_is_skipped() {
        let mut z = build_stored_zip(&[("3D/x.model", b"")]);
        let n = z.len();
        z[n - 2..].copy_from_slice(&5u16.to_le_bytes());
        z.extend_from_slice(b"hello");
        assert_eq!(names_of(&z), Some(vec!["3D/x.model".into()]));
    }
}
