# -*- coding: utf-8 -*-
"""PMX 2.0 / 2.1 parser (pure Python, no dependencies).

Returns plain dicts / lists so the data can be serialized into glTF extras
without transformation.  All values are kept in MMD's native left-handed
coordinate system; conversion happens in convert.py.
"""
import struct


class Reader:
    __slots__ = ("d", "o")

    def __init__(self, data: bytes):
        self.d = data
        self.o = 0

    def read(self, fmt):
        fmt = "<" + fmt
        v = struct.unpack_from(fmt, self.d, self.o)
        self.o += struct.calcsize(fmt)
        return v if len(v) > 1 else v[0]

    def raw(self, n):
        b = self.d[self.o:self.o + n]
        self.o += n
        return b

    def eof(self):
        return self.o >= len(self.d)


_IDX_FMT_SIGNED = {1: "b", 2: "h", 4: "i"}
_IDX_FMT_UNSIGNED = {1: "B", 2: "H", 4: "i"}  # spec: 4-byte vertex index is signed


def parse_pmx(path):
    with open(path, "rb") as f:
        data = f.read()
    r = Reader(data)

    magic = r.raw(4)
    if magic not in (b"PMX ", b"PMX\x20"):
        raise ValueError("Not a PMX file (magic=%r). PMD is not supported." % magic)
    version = round(r.read("f"), 2)
    n_globals = r.read("B")
    g = list(r.raw(n_globals))
    enc = "utf-16-le" if g[0] == 0 else "utf-8"
    add_uv = g[1]
    sz_vert, sz_tex, sz_mat, sz_bone, sz_morph, sz_rigid = g[2:8]

    def text():
        n = r.read("i")
        return r.raw(n).decode(enc, errors="replace")

    def vidx():
        return r.read(_IDX_FMT_UNSIGNED[sz_vert])

    def make_idx(size):
        fmt = _IDX_FMT_SIGNED[size]
        def rd():
            return r.read(fmt)
        return rd

    tidx = make_idx(sz_tex)
    midx = make_idx(sz_mat)
    bidx = make_idx(sz_bone)
    moidx = make_idx(sz_morph)
    ridx = make_idx(sz_rigid)

    model = {
        "version": version,
        "encoding": enc,
        "add_uv_count": add_uv,
        "name": text(), "name_en": text(),
        "comment": text(), "comment_en": text(),
    }

    # ---- vertices ------------------------------------------------------
    n = r.read("i")
    vertices = []
    for _ in range(n):
        pos = r.read("3f")
        nrm = r.read("3f")
        uv = r.read("2f")
        auv = [list(r.read("4f")) for _ in range(add_uv)]
        wt = r.read("B")
        sdef = None
        if wt == 0:      # BDEF1
            bones = [bidx()]
            weights = [1.0]
        elif wt == 1:    # BDEF2
            bones = [bidx(), bidx()]
            w = r.read("f")
            weights = [w, 1.0 - w]
        elif wt in (2, 4):  # BDEF4 / QDEF(2.1)
            bones = [bidx(), bidx(), bidx(), bidx()]
            weights = list(r.read("4f"))
        elif wt == 3:    # SDEF
            bones = [bidx(), bidx()]
            w = r.read("f")
            weights = [w, 1.0 - w]
            sdef = {"c": list(r.read("3f")),
                    "r0": list(r.read("3f")),
                    "r1": list(r.read("3f"))}
        else:
            raise ValueError("unknown weight type %d" % wt)
        edge = r.read("f")
        vertices.append({"pos": pos, "normal": nrm, "uv": uv, "add_uv": auv,
                         "weight_type": wt, "bones": bones, "weights": weights,
                         "sdef": sdef, "edge": edge})
    model["vertices"] = vertices

    # ---- faces ---------------------------------------------------------
    n = r.read("i")
    indices = list(struct.unpack_from(
        "<%d%s" % (n, _IDX_FMT_UNSIGNED[sz_vert]), r.d, r.o))
    r.o += n * sz_vert
    model["indices"] = indices

    # ---- textures ------------------------------------------------------
    n = r.read("i")
    model["textures"] = [text() for _ in range(n)]

    # ---- materials -----------------------------------------------------
    n = r.read("i")
    mats = []
    for _ in range(n):
        m = {
            "name": text(), "name_en": text(),
            "diffuse": list(r.read("4f")),
            "specular": list(r.read("3f")),
            "specular_power": r.read("f"),
            "ambient": list(r.read("3f")),
            "flags": r.read("B"),
            "edge_color": list(r.read("4f")),
            "edge_size": r.read("f"),
            "texture": tidx(),
            "sphere_texture": tidx(),
            "sphere_mode": r.read("B"),
        }
        shared_toon = r.read("B")
        if shared_toon:
            m["toon_shared"] = r.read("b")
            m["toon_texture"] = -1
        else:
            m["toon_shared"] = -1
            m["toon_texture"] = tidx()
        m["memo"] = text()
        m["index_count"] = r.read("i")
        mats.append(m)
    model["materials"] = mats

    # ---- bones ---------------------------------------------------------
    n = r.read("i")
    bones = []
    for _ in range(n):
        b = {
            "name": text(), "name_en": text(),
            "pos": list(r.read("3f")),
            "parent": bidx(),
            "layer": r.read("i"),
            "flags": r.read("H"),
        }
        fl = b["flags"]
        if fl & 0x0001:
            b["tail_bone"] = bidx()
        else:
            b["tail_offset"] = list(r.read("3f"))
        if fl & (0x0100 | 0x0200):        # inherit rotation / translation
            b["inherit_parent"] = bidx()
            b["inherit_ratio"] = r.read("f")
        if fl & 0x0400:                   # fixed axis
            b["fixed_axis"] = list(r.read("3f"))
        if fl & 0x0800:                   # local axes
            b["local_x"] = list(r.read("3f"))
            b["local_z"] = list(r.read("3f"))
        if fl & 0x2000:                   # external parent
            b["external_key"] = r.read("i")
        if fl & 0x0020:                   # IK
            links = []
            ik = {"target": bidx(), "loop": r.read("i"),
                  "limit_angle": r.read("f")}
            for _ in range(r.read("i")):
                link = {"bone": bidx()}
                if r.read("B"):
                    link["min"] = list(r.read("3f"))
                    link["max"] = list(r.read("3f"))
                links.append(link)
            ik["links"] = links
            b["ik"] = ik
        bones.append(b)
    model["bones"] = bones

    # ---- morphs --------------------------------------------------------
    n = r.read("i")
    morphs = []
    for _ in range(n):
        mo = {"name": text(), "name_en": text(),
              "panel": r.read("B"), "type": r.read("B")}
        t = mo["type"]
        cnt = r.read("i")
        offs = []
        for _ in range(cnt):
            if t == 0 or t == 9:          # group / flip(2.1)
                offs.append({"morph": moidx(), "weight": r.read("f")})
            elif t == 1:                  # vertex
                offs.append({"vertex": vidx(), "offset": list(r.read("3f"))})
            elif t == 2:                  # bone
                offs.append({"bone": bidx(), "translation": list(r.read("3f")),
                             "rotation": list(r.read("4f"))})
            elif 3 <= t <= 7:             # UV / addUV1-4
                offs.append({"vertex": vidx(), "offset": list(r.read("4f"))})
            elif t == 8:                  # material
                offs.append({
                    "material": midx(), "op": r.read("B"),
                    "diffuse": list(r.read("4f")),
                    "specular": list(r.read("3f")),
                    "specular_power": r.read("f"),
                    "ambient": list(r.read("3f")),
                    "edge_color": list(r.read("4f")),
                    "edge_size": r.read("f"),
                    "texture_tint": list(r.read("4f")),
                    "sphere_tint": list(r.read("4f")),
                    "toon_tint": list(r.read("4f")),
                })
            elif t == 10:                 # impulse (2.1)
                offs.append({"rigid": ridx(), "local": r.read("B"),
                             "velocity": list(r.read("3f")),
                             "torque": list(r.read("3f"))})
            else:
                raise ValueError("unknown morph type %d" % t)
        mo["offsets"] = offs
        morphs.append(mo)
    model["morphs"] = morphs

    # ---- display frames --------------------------------------------------
    n = r.read("i")
    frames = []
    for _ in range(n):
        fr = {"name": text(), "name_en": text(), "special": r.read("B")}
        items = []
        for _ in range(r.read("i")):
            kind = r.read("B")
            items.append({"type": "bone" if kind == 0 else "morph",
                          "index": bidx() if kind == 0 else moidx()})
        fr["items"] = items
        frames.append(fr)
    model["display_frames"] = frames

    # ---- rigid bodies ----------------------------------------------------
    n = r.read("i")
    rigids = []
    for _ in range(n):
        rigids.append({
            "name": text(), "name_en": text(),
            "bone": bidx(),
            "group": r.read("B"),
            "no_collision_mask": r.read("H"),
            "shape": r.read("B"),
            "size": list(r.read("3f")),
            "pos": list(r.read("3f")),
            "rot": list(r.read("3f")),
            "mass": r.read("f"),
            "linear_damping": r.read("f"),
            "angular_damping": r.read("f"),
            "restitution": r.read("f"),
            "friction": r.read("f"),
            "mode": r.read("B"),
        })
    model["rigid_bodies"] = rigids

    # ---- joints ----------------------------------------------------------
    n = r.read("i")
    joints = []
    for _ in range(n):
        joints.append({
            "name": text(), "name_en": text(),
            "type": r.read("B"),
            "rigid_a": ridx(), "rigid_b": ridx(),
            "pos": list(r.read("3f")), "rot": list(r.read("3f")),
            "pos_min": list(r.read("3f")), "pos_max": list(r.read("3f")),
            "rot_min": list(r.read("3f")), "rot_max": list(r.read("3f")),
            "spring_pos": list(r.read("3f")), "spring_rot": list(r.read("3f")),
        })
    model["joints"] = joints

    # PMX 2.1 soft bodies are ignored (extremely rare); remaining bytes noted.
    model["trailing_bytes"] = len(data) - r.o
    return model
