//! Sources and sinks (S12 §Interfaces, "sources / sinks").

use std::fs::File;
use std::io::{BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use crate::error::IoError;

/// Where an import reads from.
#[derive(Copy, Clone, Debug)]
pub enum Source<'a> {
    /// A file.
    Path(&'a Path),
    /// Bytes in memory.
    Bytes(&'a [u8]),
}

impl Source<'_> {
    /// Total size in bytes.
    pub fn len(&self) -> Result<u64, IoError> {
        match self {
            Source::Bytes(b) => Ok(b.len() as u64),
            Source::Path(p) => {
                std::fs::metadata(p)
                    .map(|m| m.len())
                    .map_err(|source| IoError::Read {
                        path: (*p).to_owned(),
                        source,
                    })
            }
        }
    }

    /// True for a zero-length source.
    pub fn is_empty(&self) -> Result<bool, IoError> {
        self.len().map(|n| n == 0)
    }

    /// The first `max` bytes (fewer for a shorter source) and the full length.
    pub fn head(&self, max: usize) -> Result<(Vec<u8>, u64), IoError> {
        match self {
            Source::Bytes(b) => Ok((b[..b.len().min(max)].to_vec(), b.len() as u64)),
            Source::Path(p) => {
                let err = |source| IoError::Read {
                    path: (*p).to_owned(),
                    source,
                };
                let f = File::open(p).map_err(err)?;
                let len = f.metadata().map_err(err)?.len();
                let mut buf = Vec::new();
                f.take(max as u64).read_to_end(&mut buf).map_err(err)?;
                Ok((buf, len))
            }
        }
    }

    /// Reads exactly `buf.len()` bytes at `offset`.
    pub(crate) fn read_exact_at(&self, offset: u64, buf: &mut [u8]) -> Result<(), IoError> {
        match self {
            Source::Bytes(b) => {
                let short = || IoError::Read {
                    path: PathBuf::new(),
                    source: std::io::ErrorKind::UnexpectedEof.into(),
                };
                let start = usize::try_from(offset).map_err(|_| short())?;
                let end = start.checked_add(buf.len()).ok_or_else(short)?;
                buf.copy_from_slice(b.get(start..end).ok_or_else(short)?);
                Ok(())
            }
            Source::Path(p) => {
                let err = |source| IoError::Read {
                    path: (*p).to_owned(),
                    source,
                };
                let mut f = File::open(p).map_err(err)?;
                f.seek(SeekFrom::Start(offset)).map_err(err)?;
                f.read_exact(buf).map_err(err)
            }
        }
    }

    /// Display path for diagnostics (empty for memory).
    pub fn display_path(&self) -> PathBuf {
        match self {
            Source::Path(p) => (*p).to_owned(),
            Source::Bytes(_) => PathBuf::new(),
        }
    }
}

/// Where an export writes to.
#[derive(Debug)]
pub enum Sink<'a> {
    /// Written to `<path>.tmp` in the same directory, fsynced, renamed over `path`. A failed
    /// write leaves an existing `path` untouched and removes the temporary file.
    Path(&'a Path),
    /// Appended to a buffer. A failed write truncates the buffer back to its old length.
    Memory(&'a mut Vec<u8>),
}

/// Test hook, see [`Sink::write`].
type Writer<'w> = &'w mut dyn Write;

impl Sink<'_> {
    /// Runs `f` against the sink and returns the number of bytes it wrote.
    ///
    /// For [`Sink::Path`] the bytes go to a sibling `<file name>.tmp`, which is synced and then
    /// renamed over the target, so a reader never sees a half-written file and an error from
    /// `f` (or from the disk) leaves the previous file intact.
    ///
    /// # Errors
    /// The error of `f`, or [`IoError::Write`].
    pub fn write<F>(self, f: F) -> Result<u64, IoError>
    where
        F: FnOnce(Writer<'_>) -> Result<(), IoError>,
    {
        match self {
            Sink::Memory(buf) => {
                let start = buf.len();
                match f(buf) {
                    Ok(()) => Ok((buf.len() - start) as u64),
                    Err(e) => {
                        buf.truncate(start);
                        Err(e)
                    }
                }
            }
            Sink::Path(path) => write_atomic(path, f),
        }
    }
}

/// `<path>.tmp`: the file name with `.tmp` appended, in the same directory.
pub(crate) fn tmp_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_owned();
    name.push(".tmp");
    path.with_file_name(name)
}

fn write_atomic<F>(path: &Path, f: F) -> Result<u64, IoError>
where
    F: FnOnce(Writer<'_>) -> Result<(), IoError>,
{
    let werr = |source| IoError::Write {
        path: path.to_owned(),
        source,
    };
    let tmp = tmp_path(path);
    let result = (|| {
        let file = File::create(&tmp).map_err(werr)?;
        let mut w = CountingWriter {
            inner: BufWriter::with_capacity(1 << 20, file),
            bytes: 0,
        };
        f(&mut w)?;
        w.flush().map_err(werr)?;
        let file = w.inner.into_inner().map_err(|e| werr(e.into_error()))?;
        file.sync_all().map_err(werr)?;
        let bytes = w.bytes;
        drop(file);
        std::fs::rename(&tmp, path).map_err(werr)?;
        Ok(bytes)
    })();
    if result.is_err() {
        // Best effort: the original is already safe, a stale `.tmp` is only clutter.
        let _ = std::fs::remove_file(&tmp);
    }
    result
}

struct CountingWriter<W: Write> {
    inner: W,
    bytes: u64,
}

impl<W: Write> Write for CountingWriter<W> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let n = self.inner.write(buf)?;
        self.bytes += n as u64;
        Ok(n)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_sink_writes_and_counts() {
        let dir = tempfile::tempdir().expect("tempdir");
        let p = dir.path().join("a.stl");
        let n = Sink::Path(&p)
            .write(|w| {
                w.write_all(b"hello").map_err(|source| IoError::Write {
                    path: PathBuf::new(),
                    source,
                })
            })
            .expect("write");
        assert_eq!(n, 5);
        assert_eq!(std::fs::read(&p).expect("read"), b"hello");
        assert!(!tmp_path(&p).exists());
    }

    #[test]
    fn path_sink_keeps_the_original_when_the_writer_fails_midway() {
        let dir = tempfile::tempdir().expect("tempdir");
        let p = dir.path().join("keep.3mf");
        std::fs::write(&p, b"ORIGINAL").expect("seed");
        let r = Sink::Path(&p).write(|w| {
            w.write_all(b"half of the new fi").expect("buffered");
            w.flush()
                .expect("flush so that the tmp file really has bytes");
            Err(IoError::Empty)
        });
        assert!(matches!(r, Err(IoError::Empty)));
        assert_eq!(std::fs::read(&p).expect("read"), b"ORIGINAL");
        assert!(!tmp_path(&p).exists(), "temporary file must be removed");
    }

    #[test]
    fn path_sink_failure_without_an_original_leaves_nothing() {
        let dir = tempfile::tempdir().expect("tempdir");
        let p = dir.path().join("new.stl");
        let r = Sink::Path(&p).write(|w| {
            w.write_all(b"x").expect("buffered");
            Err(IoError::Cancelled)
        });
        assert!(matches!(r, Err(IoError::Cancelled)));
        assert!(!p.exists());
        assert!(!tmp_path(&p).exists());
    }

    #[test]
    fn path_sink_replaces_an_existing_file_on_success() {
        let dir = tempfile::tempdir().expect("tempdir");
        let p = dir.path().join("x.obj");
        std::fs::write(&p, b"OLD OLD OLD").expect("seed");
        Sink::Path(&p)
            .write(|w| {
                w.write_all(b"new").map_err(|source| IoError::Write {
                    path: PathBuf::new(),
                    source,
                })
            })
            .expect("write");
        assert_eq!(std::fs::read(&p).expect("read"), b"new");
    }

    #[test]
    fn path_sink_into_a_missing_directory_is_a_write_error() {
        let dir = tempfile::tempdir().expect("tempdir");
        let p = dir.path().join("nope").join("x.stl");
        let r = Sink::Path(&p).write(|_| Ok(()));
        assert!(matches!(r, Err(IoError::Write { .. })));
    }

    #[test]
    fn memory_sink_rolls_back_on_error() {
        let mut v = b"keep".to_vec();
        let r = Sink::Memory(&mut v).write(|w| {
            w.write_all(b"junk").expect("vec");
            Err(IoError::Empty)
        });
        assert!(r.is_err());
        assert_eq!(v, b"keep");
        let n = Sink::Memory(&mut v)
            .write(|w| {
                w.write_all(b"+more").expect("vec");
                Ok(())
            })
            .expect("write");
        assert_eq!((n, v.as_slice()), (5, b"keep+more".as_slice()));
    }

    #[test]
    fn source_head_and_len() {
        let data = vec![7u8; 10_000];
        let (h, len) = Source::Bytes(&data).head(4096).expect("head");
        assert_eq!((h.len(), len), (4096, 10_000));
        let dir = tempfile::tempdir().expect("tempdir");
        let p = dir.path().join("f.bin");
        std::fs::write(&p, &data).expect("seed");
        let (h, len) = Source::Path(&p).head(4096).expect("head");
        assert_eq!((h.len(), len), (4096, 10_000));
        assert_eq!(Source::Path(&p).len().expect("len"), 10_000);
        assert!(Source::Bytes(&[]).is_empty().expect("empty"));
        let mut b = [0u8; 4];
        Source::Path(&p).read_exact_at(9_996, &mut b).expect("read");
        assert!(Source::Path(&p).read_exact_at(9_998, &mut b).is_err());
        assert!(Source::Bytes(&data).read_exact_at(9_998, &mut b).is_err());
        assert!(matches!(
            Source::Path(&dir.path().join("missing")).head(10),
            Err(IoError::Read { .. })
        ));
    }
}
