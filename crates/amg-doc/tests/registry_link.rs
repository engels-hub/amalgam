//! Done-when 4: the expected kinds are linked into a binary that only depends on `amg-doc`
//! (this is the `inventory`-on-MSVC-across-rlibs check; see `amg_doc::registry` docs).

#[test]
fn expected_kinds_are_linked() {
    amg_doc::testing::link();
    let reg = amg_doc::try_registry().expect("registry is valid");
    let kinds: Vec<&str> = reg.kinds().collect();
    for k in amg_doc::testing::KINDS {
        assert!(
            kinds.contains(&k),
            "kind `{k}` is not linked; have {kinds:?}"
        );
    }
    assert_eq!(reg.get("test.box").unwrap().version, 2);
    assert_eq!(reg.get("test.counter").unwrap().version, 1);
}

#[test]
fn base_feature_kind_is_linked() {
    let reg = amg_doc::try_registry().expect("registry is valid");
    assert_eq!(reg.get(amg_doc::BASE_FEATURE_KIND).unwrap().version, 1);
}
