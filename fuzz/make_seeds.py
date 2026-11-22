"""Writes the small committed IO-16 seed corpora into fuzz/seeds/<target>/.

Run from anywhere: `python fuzz/make_seeds.py`. Deterministic; the output is committed. Larger
real-world seeds are not copied: `run.sh` passes the existing fixture directories
(`testdata/io/heads`, `testdata/io/slicer`) to libFuzzer by path.

Seed files have no format extension, so the repository's git-LFS rules (`*.stl`, `*.3mf`, ...)
do not apply to them.
"""

import io
import random
import struct
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent / "seeds"

TET_P = [(0, 0, 0), (10, 0, 0), (0, 10, 0), (0, 0, 10)]
TET_T = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]


def put(target, name, data):
    d = ROOT / target
    d.mkdir(parents=True, exist_ok=True)
    if isinstance(data, str):
        data = data.encode("utf-8")
    (d / name).write_bytes(data)


def stl():
    facets = "".join(
        "facet normal 0 0 0\n outer loop\n"
        + "".join(f"  vertex {x} {y} {z}\n" for x, y, z in (TET_P[a], TET_P[b], TET_P[c]))
        + " endloop\nendfacet\n"
        for a, b, c in TET_T
    )
    put("stl", "ascii-tet", f"solid tet\n{facets}endsolid tet\n")
    put("stl", "ascii-multi-crlf", f"solid a\r\n{facets}endsolid a\r\nSOLID b\r\n{facets}ENDSOLID b\r\n".replace("\n", "\r\n").replace("\r\r", "\r"))
    put("stl", "ascii-quad-loop", "solid q\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 1 1 0\nvertex 0 1 0\nendloop\nendfacet\n")

    def binary(header, colors=False, count=None, extra=b""):
        recs = b""
        for k, (a, b, c) in enumerate(TET_T):
            attr = (0x8000 | (k * 0x1111 & 0x7FFF)) if colors else 0
            recs += struct.pack("<3f", 0, 0, 0)
            for v in (TET_P[a], TET_P[b], TET_P[c]):
                recs += struct.pack("<3f", *v)
            recs += struct.pack("<H", attr)
        n = len(TET_T) if count is None else count
        return header.ljust(80, b"\0")[:80] + struct.pack("<I", n) + recs + extra

    put("stl", "binary-tet", binary(b"Amalgam seed"))
    put("stl", "binary-solid-header", binary(b"solid looks like ascii"))
    put("stl", "binary-colors", binary(b"COLOR=\x80\x80\x80\xff", colors=True))
    put("stl", "binary-wrong-count", binary(b"x", count=9, extra=b"\0" * 7))
    nan = binary(b"nan")
    nan = nan[:84 + 12] + struct.pack("<f", float("nan")) + nan[84 + 16:]
    put("stl", "binary-nan", nan)


def obj():
    put("obj", "tet", "".join(f"v {x} {y} {z}\n" for x, y, z in TET_P) + "".join(f"f {a+1} {b+1} {c+1}\n" for a, b, c in TET_T))
    put("obj", "mixed", """# comment
mtllib m.mtl
o first
v 0 0 0 1 0 0
v 1 0 0 0 1 0
v 1 1 0 0 0 1
v 0 1 0 \\
  0.5 0.5 0.5
vt 0 0
vt 1 0
vt 1 1
vn 0 0 1
usemtl red
f 1/1/1 2/2/1 3/3/1
f -4//1 -2//1 -1//1
g second
usemtl blue
f 1 2 3 4
o empty
""")
    put("obj", "concave-polygon", "v 0 0 0\nv 4 0 0\nv 4 4 0\nv 2 1 0\nv 0 4 0\nf 1 2 3 4 5\nf 1 1 2\nf 1 2 9\n")
    put("obj", "crlf-bom", "﻿v 0 0 0\r\nv 1 0 0\r\nv 0 1 0\r\nf 1 2 3\r\n")


def mtl():
    put("mtl", "basic", """newmtl red
Kd 1 0 0
d 0.5
newmtl blue
Kd 0 0 1
Tr 0.25
map_Kd tex.png
newmtl   spaced name  \r
Kd 0.2 0.3
bump b.png
""")


def ply():
    ascii = """ply
format ascii 1.0
comment seed
element vertex 4
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
element face 4
property list uchar int vertex_indices
end_header
""" + "".join(f"{x} {y} {z} 255 0 0\n" for x, y, z in TET_P) + "".join(f"3 {a} {b} {c}\n" for a, b, c in TET_T)
    put("ply", "ascii-tet", ascii)
    for order, fmt in (("little", "<"), ("big", ">")):
        head = f"""ply
format binary_{order}_endian 1.0
element vertex 4
property double x
property double y
property double z
element face 4
property list uchar uint vertex_indices
property float s
element extra 1
property ushort q
end_header
""".encode()
        body = b"".join(struct.pack(fmt + "3d", *p) for p in TET_P)
        body += b"".join(struct.pack(fmt + "B3If", 3, a, b, c, 0.5) for a, b, c in TET_T)
        body += struct.pack(fmt + "H", 7)
        put("ply", f"binary-{order}", head + body)
    put("ply", "point-cloud", "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 1 1\n")


CORE = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
MAT = "http://schemas.microsoft.com/3dmanufacturing/material/2015/02"
PROD = "http://schemas.microsoft.com/3dmanufacturing/production/2015/06"


def mesh_xml(extra_tri=lambda k: ""):
    v = "".join(f'<vertex x="{x}" y="{y}" z="{z}"/>' for x, y, z in TET_P)
    t = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"{extra_tri(k)}/>' for k, (a, b, c) in enumerate(TET_T))
    return f"<mesh><vertices>{v}</vertices><triangles>{t}</triangles></mesh>"


def models():
    basic = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="{CORE}">
<metadata name="Title">Seed &amp; tet</metadata>
<resources><object id="1" type="model" name="tet">{mesh_xml()}</object></resources>
<build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 5 5 0"/></build>
</model>"""
    materials = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="inch" xmlns="{CORE}" xmlns:m="{MAT}">
<resources>
<basematerials id="1"><base name="Red" displaycolor="#FF0000"/><base name="Blue" displaycolor="#0000FFC0"/></basematerials>
<m:colorgroup id="2"><m:color color="#00FF00"/><m:color color="#123456"/></m:colorgroup>
<object id="3" type="model" pid="1" pindex="0">{mesh_xml(lambda k: ' pid="2" p1="1"' if k == 1 else (' p1="1"' if k == 2 else ""))}</object>
<object id="4" type="support">{mesh_xml()}</object>
<object id="5"><components><component objectid="3" transform="1 0 0 0 1 0 0 0 1 0 0 20"/><component objectid="4"/></components></object>
</resources>
<build><item objectid="5"/><item objectid="3" transform="0 1 0 -1 0 0 0 0 1 0 0 0"/></build>
</model>"""
    paint = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="{CORE}" xmlns:slic3rpe="http://schemas.slic3r.org/3mf/2017/06" requiredextensions="">
<resources><object id="1" type="model">{mesh_xml(lambda k: [' paint_color="4"', ' paint_color="0C"', ' slic3rpe:mmu_segmentation="841"', ' paint_color="0FC"'][k])}</object></resources>
<build><item objectid="1"/></build>
</model>"""
    production = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="centimeter" xmlns="{CORE}" xmlns:p="{PROD}" requiredextensions="p">
<resources><object id="1" type="model" p:UUID="11111111-1111-4111-8111-111111111111">{mesh_xml()}</object>
<object id="2" p:UUID="22222222-2222-4222-8222-222222222222"><components><component objectid="1" p:path="/3D/3dmodel.model"/></components></object></resources>
<build p:UUID="33333333-3333-4333-8333-333333333333"><item objectid="2" p:UUID="44444444-4444-4444-8444-444444444444"/></build>
</model>"""
    return {"basic": basic, "materials": materials, "paint": paint, "production": production}


def threemf():
    for name, xml in models().items():
        put("threemf_model", name, xml)
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    for name, method in (("stored", zipfile.ZIP_STORED), ("deflated", zipfile.ZIP_DEFLATED)):
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", method) as z:
            for n, data in (("[Content_Types].xml", ct), ("_rels/.rels", rels), ("3D/3dmodel.model", models()["materials"])):
                zi = zipfile.ZipInfo(n, date_time=(1980, 1, 1, 0, 0, 0))
                zi.compress_type = method
                z.writestr(zi, data)
        put("threemf_package", f"materials-{name}", buf.getvalue())


def paint():
    for k, s in enumerate(["", "4", "8", "0C", "1C", "DC", "EC", "0FC", "FFC", "841", "C8C41", "48483", "3", "G"]):
        put("paint_decode", f"s{k:02}", s)


def roundtrip():
    rng = random.Random(16)
    for n in (8, 64, 400, 2000):
        put("roundtrip_scene", f"random-{n}", bytes(rng.randrange(256) for _ in range(n)))


if __name__ == "__main__":
    stl()
    obj()
    mtl()
    ply()
    threemf()
    paint()
    roundtrip()
    (ROOT / ".gitattributes").write_bytes(b"# Byte-exact seeds: no line-ending conversion, no git-LFS.\n* -text -filter -diff -merge\n")
