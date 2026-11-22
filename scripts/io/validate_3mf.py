#!/usr/bin/env python3
"""Validate 3MF packages written by Amalgam (S12 IO-11, Done-when 2).

For every file given on the command line (or every *.3mf in a directory given):

1. lib3mf (PyPI `lib3mf`, the 3MF Consortium library) reads it in strict mode
   (`SetStrictModeActive(True)`); any warning counts as a failure.
2. Every model part (`*.model`), `[Content_Types].xml` and every `.rels` part is validated
   with `xmlschema` against the 3MF Consortium consolidated schema (test_suites,
   `Consolidated_Schema/Schema`, pinned commit below).

The schema is read from `--schema-dir` or `AMG_3MF_SCHEMA_DIR`; if neither is given it is
downloaded once into a per-user cache (`%LOCALAPPDATA%/amalgam/3mf-schema/<commit>` or
`~/.cache/amalgam/3mf-schema/<commit>`) and every file is checked against the SHA-256 below.

Prints `OK <file>` or `FAIL <file>: <reason>` per file; the exit status is 1 if any failed.

Setup (dev only, outside the repository):
    python -m venv <venv>
    <venv>/Scripts/python -m pip install lib3mf==2.5.0 xmlschema==4.3.2
    <venv>/Scripts/python scripts/io/validate_3mf.py out/*.3mf
"""

from __future__ import annotations

import argparse
import hashlib
import io
import os
import pathlib
import sys
import urllib.request
import zipfile

SCHEMA_COMMIT = "f483d3beee069e7adbac125d1f7baa36b5b3e5bf"
SCHEMA_URL = (
    "https://raw.githubusercontent.com/3MFConsortium/test_suites/"
    + SCHEMA_COMMIT
    + "/Consolidated_Schema/Schema/"
)
SCHEMA_FILES = {
    "opc-contentTypes.xsd": "9e0b7209fc69ab11987900404540969976000c5ebe4d4f58c43dc3842886bf3a",
    "opc-coreProperties.xsd": "451958454e8588dfc7cd945981ada142ca06ff3307937f5700df059c2b307fa8",
    "opc-relationships.xsd": "f565adfef5a502044abc3a9153e157edc25af78304d335994afb958874b15e26",
    "qli_3MF.xsd": "7ef9331a0a68ecbd345f1f6744d6f126b3d3b3a4a7d6ca3c4dd4a29afa3aee60",
    "qli_SecureContent.xsd": "76acd96fc83f4cc87db94511473d3c7694d7069ce9790dd1487ac00ab9d1baa6",
    "qli_balls_import.xsd": "8e40ef6c16ba8734ed351138c1f186c439388292a587ccf0814432c293968a72",
    "qli_beamlattice_import.xsd": "3f1489d11f7de7a54b8ff1de3c74e47c139c1017533b56861c271387a0fdd892",
    "qli_booleans_import.xsd": "d1e98ff58b7fcb49714af2432dc7cce28f812c55d9589238ae3ba0f905427d44",
    "qli_core_import.xsd": "186db623e8d6ff57324c3d4cd301baf034e67e24245b7ab732ad7be498a41d0c",
    "qli_displacement_import.xsd": "6851c495b0374ad1825bfd0e2cec4df83ee3c76e25c2752962b2e52673d6fe95",
    "qli_implicit_import.xsd": "974a2bbb568ab095e276449dbdfb4a987905b9bceaa2f59565f08b60d8780415",
    "qli_material.xsd": "be26fa6205ef1b98bfb6016a5f12973c2724ad26a1801ef08c70703df20c18eb",
    "qli_printticket.xsd": "94791b25bb19c5e145da20bc3877c14278dcab8041e076e828390579d7a46f16",
    "qli_production_alternative_import.xsd": "1bafe90f2fd956561ef9896c686ec6796d5773cb30381b81e2b34bc4996b4f52",
    "qli_production_import.xsd": "a129f203879f8375061f6e3ff5b5402c840086da8b3b52ca6ba452072f550ee6",
    "qli_slice_import.xsd": "4e98ef2076c229c93aed32da560fc5cd1c545e2802521d834b2f36e5b23901c3",
    "qli_triangleset.xsd": "0da4dc20c33798e8724d47c4bd58ebe4beba0e4953eb60a711baec21850bce6e",
    "qli_volumetric_import.xsd": "a528ef34b8e419ff0c6e5355f138870ed8a16ca9673554d055c7d5ca4f23c054",
    "qli_xenc_core_import.xsd": "1e240bc9e77b46d492318761a5f88f501535de9b0377c3cd150e88ce74fb9011",
    "qli_xmldsig_import.xsd": "0a1c72881eee256a8cc80bfcb5d98b309760a368c886ef26eda65df546f4cf76",
    "xml.xsd": "bff1de4be4a376a0e2acc1d01c1f103d6baa15417aaadeffcfe3af25719d61f5",
}


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def default_cache() -> pathlib.Path:
    base = os.environ.get("LOCALAPPDATA")
    root = pathlib.Path(base) if base else pathlib.Path.home() / ".cache"
    return root / "amalgam" / "3mf-schema" / SCHEMA_COMMIT[:8]


def schema_dir(arg: str | None) -> pathlib.Path:
    given = arg or os.environ.get("AMG_3MF_SCHEMA_DIR")
    d = pathlib.Path(given) if given else default_cache()
    d.mkdir(parents=True, exist_ok=True)
    for name, digest in SCHEMA_FILES.items():
        p = d / name
        if not p.exists():
            if given:
                raise SystemExit(f"schema file {p} is missing")
            with urllib.request.urlopen(SCHEMA_URL + name, timeout=60) as r:
                p.write_bytes(r.read())
        if sha256(p) != digest:
            raise SystemExit(f"schema file {p} does not match its pinned SHA-256")
    return d


class Schemas:
    def __init__(self, d: pathlib.Path) -> None:
        import xmlschema

        self.model = xmlschema.XMLSchema(str(d / "qli_3MF.xsd"))
        self.content_types = xmlschema.XMLSchema(str(d / "opc-contentTypes.xsd"))
        self.rels = xmlschema.XMLSchema(str(d / "opc-relationships.xsd"))


def xsd_errors(schemas: Schemas, package: zipfile.ZipFile) -> list[str]:
    errors: list[str] = []
    for info in package.infolist():
        name = info.filename
        if name.endswith(".model"):
            schema = schemas.model
        elif name == "[Content_Types].xml":
            schema = schemas.content_types
        elif name.endswith(".rels"):
            schema = schemas.rels
        else:
            continue
        data = package.read(info)
        for e in schema.iter_errors(io.BytesIO(data)):
            errors.append(f"{name}: XSD: {e.reason} at {e.path}")
            if len(errors) > 20:
                return errors
    return errors


def lib3mf_problems(path: pathlib.Path) -> list[str]:
    import lib3mf

    wrapper = lib3mf.get_wrapper()
    model = wrapper.CreateModel()
    reader = model.QueryReader("3mf")
    reader.SetStrictModeActive(True)
    problems: list[str] = []
    try:
        reader.ReadFromFile(str(path))
    except Exception as e:  # lib3mf raises ELib3MFException
        problems.append(f"lib3mf: {e}")
        return problems
    for i in range(reader.GetWarningCount()):
        text, code = reader.GetWarning(i)
        problems.append(f"lib3mf warning {code}: {text}")
    if model.GetBuildItems().Count() == 0:
        problems.append("lib3mf: no build items")
    return problems


def files_of(args: list[str]) -> list[pathlib.Path]:
    out: list[pathlib.Path] = []
    for a in args:
        p = pathlib.Path(a)
        if p.is_dir():
            out.extend(sorted(p.glob("*.3mf")))
        else:
            out.append(p)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+", help="3MF files or directories of them")
    ap.add_argument("--schema-dir", help="folder with the consolidated schema")
    args = ap.parse_args()
    try:
        import lib3mf  # noqa: F401
        import xmlschema  # noqa: F401
    except ImportError as e:
        print(f"missing Python package: {e.name} (pip install lib3mf xmlschema)", file=sys.stderr)
        return 2
    schemas = Schemas(schema_dir(args.schema_dir))
    failed = 0
    files = files_of(args.files)
    if not files:
        print("no 3MF files given", file=sys.stderr)
        return 2
    for path in files:
        try:
            with zipfile.ZipFile(path) as z:
                problems = xsd_errors(schemas, z)
        except zipfile.BadZipFile as e:
            problems = [f"zip: {e}"]
        problems += lib3mf_problems(path)
        if problems:
            failed += 1
            print(f"FAIL {path}: " + "; ".join(problems))
        else:
            print(f"OK {path}")
    import lib3mf as l3

    major, minor, micro = l3.get_wrapper().GetLibraryVersion()
    print(f"lib3mf {major}.{minor}.{micro}, schema test_suites@{SCHEMA_COMMIT[:8]}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
