# -*- coding: utf-8 -*-
"""Structural sanity checker for GLB files produced by mmd2gltf."""
import json
import struct
import sys

CSIZE = {5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT2": 4,
         "MAT3": 9, "MAT4": 16}


def fail(msg):
    print("FAIL:", msg)
    sys.exit(1)


def check(path):
    with open(path, "rb") as f:
        d = f.read()
    magic, ver, total = struct.unpack_from("<4sII", d, 0)
    assert magic == b"glTF" and ver == 2, "bad header"
    assert total == len(d), "length mismatch"
    ln, typ = struct.unpack_from("<I4s", d, 12)
    assert typ == b"JSON"
    j = json.loads(d[20:20 + ln])
    bo = 20 + ln
    bln, btyp = struct.unpack_from("<I4s", d, bo)
    assert btyp == b"BIN\x00"
    binbuf = d[bo + 8:bo + 8 + bln]
    assert j["buffers"][0]["byteLength"] == len(binbuf)

    # buffer views in range
    for i, bv in enumerate(j.get("bufferViews", [])):
        if bv.get("byteOffset", 0) + bv["byteLength"] > len(binbuf):
            fail("bufferView %d out of range" % i)
        if bv.get("byteOffset", 0) % 4:
            fail("bufferView %d misaligned" % i)

    # accessors fit
    for i, a in enumerate(j.get("accessors", [])):
        size = CSIZE[a["componentType"]] * NCOMP[a["type"]] * a["count"]
        if "bufferView" in a:
            bv = j["bufferViews"][a["bufferView"]]
            if a.get("byteOffset", 0) + size > bv["byteLength"]:
                fail("accessor %d overflows bufferView" % i)
        sp = a.get("sparse")
        if sp:
            iv = j["bufferViews"][sp["indices"]["bufferView"]]
            if sp["count"] * CSIZE[sp["indices"]["componentType"]] > iv["byteLength"]:
                fail("sparse indices overflow (accessor %d)" % i)
            vv = j["bufferViews"][sp["values"]["bufferView"]]
            if sp["count"] * CSIZE[a["componentType"]] * NCOMP[a["type"]] > vv["byteLength"]:
                fail("sparse values overflow (accessor %d)" % i)

    def acc_data(ai):
        a = j["accessors"][ai]
        bv = j["bufferViews"][a["bufferView"]]
        off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        n = a["count"] * NCOMP[a["type"]]
        fmt = {5126: "f", 5125: "I", 5123: "H", 5121: "B"}[a["componentType"]]
        return a, struct.unpack_from("<%d%s" % (n, fmt), binbuf, off)

    mesh = j["meshes"][0]
    nverts = j["accessors"][mesh["primitives"][0]["attributes"]["POSITION"]]["count"]
    njoints = len(j["skins"][0]["joints"])
    for p, prim in enumerate(mesh["primitives"]):
        _, idx = acc_data(prim["indices"])
        if max(idx) >= nverts:
            fail("prim %d index out of range" % p)
        _, jt = acc_data(prim["attributes"]["JOINTS_0"])
        if max(jt) >= njoints:
            fail("prim %d joint out of range" % p)
        _, wt = acc_data(prim["attributes"]["WEIGHTS_0"])
        for v in range(nverts):
            s = sum(wt[v * 4:v * 4 + 4])
            if abs(s - 1.0) > 1e-3:
                fail("weights not normalized at vertex %d (%f)" % (v, s))
        for t, tgt in enumerate(prim.get("targets", [])):
            for attr, ai in tgt.items():
                a = j["accessors"][ai]
                if a["count"] != nverts:
                    fail("target %d %s count mismatch" % (t, attr))
                sp = a.get("sparse")
                if sp:
                    iv = j["bufferViews"][sp["indices"]["bufferView"]]
                    off = iv.get("byteOffset", 0)
                    sidx = struct.unpack_from("<%dI" % sp["count"], binbuf, off)
                    if list(sidx) != sorted(set(sidx)):
                        fail("sparse indices not strictly increasing")
                    if max(sidx) >= nverts:
                        fail("sparse index out of range")

    # node graph sanity
    seen = set()
    def walk(ni):
        if ni in seen:
            fail("node cycle at %d" % ni)
        seen.add(ni)
        for c in j["nodes"][ni].get("children", []):
            walk(c)
    for s in j["scenes"][0]["nodes"]:
        walk(s)

    # animations
    for an in j.get("animations", []):
        for ch in an["channels"]:
            sa = an["samplers"][ch["sampler"]]
            ia = j["accessors"][sa["input"]]
            oa = j["accessors"][sa["output"]]
            path_ = ch["target"]["path"]
            if path_ == "rotation" and oa["type"] != "VEC4":
                fail("rotation output not VEC4")
            if path_ == "translation" and oa["type"] != "VEC3":
                fail("translation output not VEC3")
            if path_ == "weights":
                ntargets = len(mesh["primitives"][0]["targets"])
                if oa["count"] != ia["count"] * ntargets:
                    fail("weights output count mismatch")
            elif oa["count"] != ia["count"]:
                fail("sampler in/out count mismatch")
            _, times = acc_data(sa["input"])
            if list(times) != sorted(times):
                fail("animation input not sorted")
            if path_ == "rotation":
                _, q = acc_data(sa["output"])
                for k in range(0, len(q), 4):
                    n = sum(c * c for c in q[k:k + 4]) ** 0.5
                    if abs(n - 1) > 1e-3:
                        fail("non-unit quaternion in animation")

    print("OK:", path)
    print("  nodes=%d meshes=%d prims=%d verts=%d joints=%d" % (
        len(j["nodes"]), len(j["meshes"]), len(mesh["primitives"]),
        nverts, njoints))
    print("  targets=%s" % mesh.get("extras", {}).get("targetNames"))
    print("  materials=%d images=%d" % (len(j.get("materials", [])),
                                        len(j.get("images", []))))
    for an in j.get("animations", []):
        print("  animation '%s': %d channels, %d samplers" % (
            an.get("name"), len(an["channels"]), len(an["samplers"])))
    if "extras" in j and "mmd" in j["extras"]:
        mm = j["extras"]["mmd"]
        print("  extras.mmd: rigidBodies=%d joints=%d morphs=%d frames=%d" % (
            len(mm["rigidBodies"]), len(mm["joints"]),
            len(mm["morphs"]), len(mm["displayFrames"])))


if __name__ == "__main__":
    check(sys.argv[1])
