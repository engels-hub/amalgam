//! Unit tests of the PLY reader: the result does not depend on chunking, block size or the
//! thread count.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test code; a failed expectation is a test failure"
)]

use amg_jobs::JobCtx;

use super::{Tuning, read_tuned};
use crate::endpoint::Source;
use crate::options::ImportOptions;

/// Vertices with colours and s/t, triangles, quads, a face colour, extra elements.
fn sample(binary: bool) -> Vec<u8> {
    let nv = 200usize;
    let mut out = format!(
        "ply\nformat {} 1.0\nelement vertex {nv}\nproperty float x\nproperty double y\nproperty float z\n\
         property uchar red\nproperty uchar green\nproperty uchar blue\nproperty float s\nproperty float t\n\
         element edge 3\nproperty int a\nproperty list uchar int b\n\
         element face {}\nproperty list uchar int vertex_indices\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n\
         end_header\n",
        if binary { "binary_little_endian" } else { "ascii" },
        nv - 3 + nv / 4
    )
    .into_bytes();
    let vert = |i: usize| {
        let f = i as f64;
        (
            (f * 0.37).sin() as f32 * 10.0,
            (f * 0.11).cos() * 7.0,
            (f * 0.05) as f32,
            [(i % 251) as u8, (i * 3 % 251) as u8, (i * 5 % 251) as u8],
            [i as f32 / 200.0, 1.0 - i as f32 / 200.0],
        )
    };
    for i in 0..nv {
        let (x, y, z, c, st) = vert(i);
        if binary {
            out.extend_from_slice(&x.to_le_bytes());
            out.extend_from_slice(&y.to_le_bytes());
            out.extend_from_slice(&z.to_le_bytes());
            out.extend_from_slice(&c);
            out.extend_from_slice(&st[0].to_le_bytes());
            out.extend_from_slice(&st[1].to_le_bytes());
        } else {
            out.extend_from_slice(
                format!(
                    "{x} {y} {z} {} {} {} {} {}\n",
                    c[0], c[1], c[2], st[0], st[1]
                )
                .as_bytes(),
            );
        }
    }
    for k in 0..3i32 {
        if binary {
            out.extend_from_slice(&k.to_le_bytes());
            out.push(2);
            out.extend_from_slice(&1i32.to_le_bytes());
            out.extend_from_slice(&2i32.to_le_bytes());
        } else {
            out.extend_from_slice(format!("{k} 2 1 2\n").as_bytes());
        }
    }
    let mut face = |corners: &[usize], c: [u8; 3]| {
        if binary {
            out.push(corners.len() as u8);
            for &i in corners {
                out.extend_from_slice(&(i as i32).to_le_bytes());
            }
            out.extend_from_slice(&c);
        } else {
            let list: Vec<String> = corners.iter().map(ToString::to_string).collect();
            out.extend_from_slice(
                format!(
                    "{} {} {} {} {}\n",
                    corners.len(),
                    list.join(" "),
                    c[0],
                    c[1],
                    c[2]
                )
                .as_bytes(),
            );
        }
    };
    for i in 0..nv - 3 {
        face(&[i, i + 1, i + 2], [(i % 3) as u8 * 100, 5, 6]);
    }
    for i in (0..nv - 4).step_by(4) {
        face(&[i, i + 1, i + 2, i + 3], [9, 9, 9]);
    }
    out
}

fn fingerprint(bytes: &[u8], tuning: Tuning) -> String {
    let i = read_tuned(
        Source::Bytes(bytes),
        &ImportOptions::default(),
        &JobCtx::detached(),
        tuning,
    )
    .expect("read");
    let o = &i.scene.objects[0];
    format!(
        "{:?}\n{:?}\n{:?}\n{:?}\n{:?}\n{:?}\n{:?}",
        o.mesh.positions,
        o.mesh.tris,
        o.mesh.attrs.vertex_colors,
        o.mesh.attrs.corner_uvs,
        o.face_materials,
        i.scene.materials,
        i.report.warnings
    )
}

#[test]
fn chunking_and_blocking_do_not_change_the_result() {
    for binary in [true, false] {
        let bytes = sample(binary);
        let reference = fingerprint(&bytes, Tuning::default());
        for chunk_bytes in [1, 3, 7, 64, 1000, 1 << 20] {
            for block_records in [1, 2, 33, 1 << 15] {
                if chunk_bytes == 1 && block_records > 2 {
                    continue;
                }
                let t = Tuning {
                    chunk_bytes,
                    block_records,
                };
                assert_eq!(
                    fingerprint(&bytes, t),
                    reference,
                    "binary={binary} chunk={chunk_bytes} block={block_records}"
                );
            }
        }
    }
}

#[test]
fn ascii_and_binary_agree() {
    let a = fingerprint(&sample(false), Tuning::default());
    let b = fingerprint(&sample(true), Tuning::default());
    assert_eq!(a, b);
}

#[test]
fn thread_count_does_not_change_the_result() {
    let bytes = sample(true);
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool")
            .install(|| {
                fingerprint(
                    &bytes,
                    Tuning {
                        chunk_bytes: 500,
                        block_records: 3,
                    },
                )
            })
    };
    assert_eq!(run(1), run(16));
}

#[test]
fn a_cancelled_job_stops() {
    let ctx = JobCtx::detached();
    ctx.cancel_token().cancel();
    let e = read_tuned(
        Source::Bytes(&sample(true)),
        &ImportOptions::default(),
        &ctx,
        Tuning::default(),
    )
    .expect_err("cancelled");
    assert!(matches!(e, crate::IoError::Cancelled), "{e:?}");
}
