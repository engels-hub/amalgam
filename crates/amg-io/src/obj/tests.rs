//! Chunking and thread-count independence of the reader (S12 §13).

use amg_jobs::JobCtx;
use proptest::prelude::*;

use super::{Tuning, read_tuned};
use crate::{ImportOptions, ObjGroups, Source};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
}

/// A syntactically valid OBJ with a bit of everything, from a seed.
fn generate(seed: u64, statements: usize) -> String {
    let mut r = Rng(seed | 1);
    let (mut nv, mut nvt) = (0u64, 0u64);
    let mut s = String::new();
    let eol = |r: &mut Rng| match r.below(6) {
        0 => " \\r\n  ",
        1 => " \\n",
        _ => "",
    };
    for _ in 0..statements {
        match r.below(12) {
            0..=3 => {
                let c = r.below(8) == 0;
                let mut line = format!(
                    "v {} {} {}",
                    r.below(1000) as f64 / 8.0,
                    r.below(1000) as f64 / 8.0,
                    r.below(1000) as f64 / 8.0
                );
                if c {
                    line.push_str(&format!(" {} 0.5 0.25", r.below(2)));
                }
                s.push_str(&line);
                nv += 1;
            }
            4 => {
                s.push_str(&format!(
                    "vt {} {}",
                    r.below(100) as f64 / 100.0,
                    r.below(100) as f64 / 100.0
                ));
                nvt += 1;
            }
            5..=8 if nv >= 3 => {
                let n = 3 + r.below(4) as usize;
                s.push('f');
                let with_vt = nvt > 0 && r.below(2) == 0;
                for _ in 0..n {
                    let rel = r.below(3) == 0;
                    let idx = |r: &mut Rng, count: u64| -> String {
                        let k = 1 + r.below(count);
                        if rel {
                            format!("-{}", count - k + 1)
                        } else {
                            k.to_string()
                        }
                    };
                    let v = idx(&mut r, nv);
                    s.push(' ');
                    s.push_str(&v);
                    if with_vt {
                        s.push('/');
                        let t = idx(&mut r, nvt);
                        s.push_str(&t);
                    }
                    s.push_str(eol(&mut r));
                }
            }
            9 => s.push_str(&format!("o obj{}", r.below(4))),
            10 => s.push_str(&format!("g grp{}", r.below(3))),
            _ => s.push_str(&format!("usemtl mat{}", r.below(3))),
        }
        s.push_str(if r.below(4) == 0 { "\r\n" } else { "\n" });
        if r.below(10) == 0 {
            s.push_str("# comment\n\n");
        }
    }
    s
}

fn read_in(
    text: &[u8],
    threads: usize,
    chunk: usize,
    block: usize,
    mode: ObjGroups,
) -> Result<crate::Imported, crate::IoError> {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool");
    let mut opts = ImportOptions::default();
    opts.obj.groups_as = mode;
    pool.install(|| {
        read_tuned(
            Source::Bytes(text),
            &opts,
            &JobCtx::detached(),
            Tuning {
                chunk_bytes: chunk,
                min_block: block,
            },
        )
    })
}

fn same(a: &crate::Imported, b: &crate::Imported) -> Result<(), TestCaseError> {
    prop_assert_eq!(a.scene.objects.len(), b.scene.objects.len());
    prop_assert_eq!(&a.scene.materials, &b.scene.materials);
    prop_assert_eq!(&a.report.warnings, &b.report.warnings);
    for (x, y) in a.scene.objects.iter().zip(&b.scene.objects) {
        prop_assert_eq!(&x.name, &y.name);
        prop_assert_eq!(&*x.mesh, &*y.mesh);
        prop_assert_eq!(&x.face_materials, &y.face_materials);
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(80))]

    /// The scene does not depend on the chunk size, block size or thread count.
    #[test]
    fn result_is_independent_of_chunking_and_threads(
        seed in any::<u64>(),
        statements in 5usize..400,
        chunk in 8usize..700,
        block in 1usize..200,
        mode in prop::sample::select(vec![ObjGroups::Objects, ObjGroups::FaceGroups, ObjGroups::Ignore]),
    ) {
        let text = generate(seed, statements);
        let reference = read_in(text.as_bytes(), 1, 1 << 20, 1 << 20, mode);
        let chunked = read_in(text.as_bytes(), 16, chunk, block, mode);
        match (reference, chunked) {
            (Ok(a), Ok(b)) => same(&a, &b)?,
            (Err(a), Err(b)) => prop_assert_eq!(a.to_string(), b.to_string()),
            (a, b) => prop_assert!(false, "{a:?} vs {b:?}"),
        }
    }
}
