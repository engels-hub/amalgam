//! In-memory ring of the most recent log lines, for crash reports.

use std::{collections::VecDeque, io, time::Duration};

use parking_lot::Mutex;

use super::RING_LINES;

static RING: Mutex<VecDeque<String>> = Mutex::new(VecDeque::new());

/// Pushes complete lines into the ring buffer, keeping at most [`RING_LINES`].
pub(crate) fn push_lines(text: &str) {
    let mut ring = RING.lock();
    for line in text.lines() {
        if ring.len() == RING_LINES {
            ring.pop_front();
        }
        ring.push_back(line.to_owned());
    }
}

/// The buffered lines, oldest first. Gives up (returns `None`) if the ring stays locked, e.g.
/// because the panicking thread holds it.
pub(crate) fn snapshot() -> Option<Vec<String>> {
    RING.try_lock_for(Duration::from_millis(200))
        .map(|ring| ring.iter().cloned().collect())
}

/// A `tracing_subscriber` writer: buffers one formatted event, pushes it into the ring on drop.
#[derive(Debug, Default)]
pub(crate) struct RingWriter {
    buf: Vec<u8>,
}

impl RingWriter {
    /// `MakeWriter` constructor (a `fn() -> impl Write` is a `MakeWriter`).
    pub(crate) fn make() -> Self {
        Self::default()
    }
}

impl io::Write for RingWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.buf.extend_from_slice(data);
        Ok(data.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for RingWriter {
    fn drop(&mut self) {
        if !self.buf.is_empty() {
            push_lines(&String::from_utf8_lossy(&self.buf));
        }
    }
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use super::*;

    #[test]
    fn ring_keeps_the_last_lines() {
        for i in 0..(RING_LINES + 50) {
            let mut w = RingWriter::make();
            write!(w, "ring-test line {i}\nsecond {i}\n").unwrap();
        }
        let snap = snapshot().unwrap();
        assert_eq!(snap.len(), RING_LINES);
        // Other tests may log concurrently; our final lines are the newest of their kind.
        let ours: Vec<_> = snap.iter().filter(|l| l.starts_with("second")).collect();
        assert_eq!(
            ours.last().map(|s| s.as_str()),
            Some(format!("second {}", RING_LINES + 49).as_str())
        );
    }
}
