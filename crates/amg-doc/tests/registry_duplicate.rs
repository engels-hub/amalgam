//! Done-when 3: a duplicate kind fails at startup. This binary registers two types under the
//! same kind (plus the test kinds, via the `test-features` dev-dependency).

use amg_doc::{Dependency, Feature, FeatureError, FeatureOutput, RecomputeCtx, register_feature};
use serde::{Deserialize, Serialize};

macro_rules! dummy {
    ($name:ident) => {
        #[derive(Clone, Debug, Serialize, Deserialize)]
        struct $name {}
        impl Feature for $name {
            fn inputs(&self) -> Vec<Dependency> {
                Vec::new()
            }
            fn compute(&self, _: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
                Ok(FeatureOutput::default())
            }
        }
    };
}

dummy!(First);
dummy!(Second);
register_feature!(First, "dup.kind", version = 1);
register_feature!(Second, "dup.kind", version = 1);

#[test]
fn duplicate_kind_is_a_startup_error() {
    let err = amg_doc::try_registry().expect_err("duplicate kind must be reported");
    assert_eq!(*err, amg_doc::RegistryError::DuplicateKind("dup.kind"));
    let panicked = std::panic::catch_unwind(amg_doc::registry).is_err();
    assert!(panicked, "registry() must panic on a broken registry");
}
