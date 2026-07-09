# -*- coding: utf-8 -*-
"""Minimal glTF 2.0 / GLB builder."""
import json
import struct

FLOAT = 5126
UINT = 5125
USHORT = 5123
UBYTE = 5121
ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963

_NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
_FMT = {FLOAT: "f", UINT: "I", USHORT: "H", UBYTE: "B"}


class GltfBuilder:
    def __init__(self, generator="mmd2gltf"):
        self.bin = bytearray()
        self.j = {
            "asset": {"version": "2.0", "generator": generator},
            "buffers": [],
            "bufferViews": [],
            "accessors": [],
        }

    # ---- binary ---------------------------------------------------------
    def add_bv(self, data: bytes, target=None, byte_stride=None) -> int:
        while len(self.bin) % 4:
            self.bin.append(0)
        bv = {"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(data)}
        if target:
            bv["target"] = target
        if byte_stride:
            bv["byteStride"] = byte_stride
        self.bin += data
        self.j["bufferViews"].append(bv)
        return len(self.j["bufferViews"]) - 1

    def add_accessor(self, flat, comp_type, type_, target=None, minmax=False,
                     normalized=False):
        """flat: flat list of numbers."""
        nc = _NCOMP[type_]
        count = len(flat) // nc
        data = struct.pack("<%d%s" % (len(flat), _FMT[comp_type]), *flat)
        bv = self.add_bv(data, target)
        acc = {"bufferView": bv, "componentType": comp_type,
               "count": count, "type": type_}
        if normalized:
            acc["normalized"] = True
        if minmax and count:
            mins = list(flat[:nc])
            maxs = list(flat[:nc])
            for i in range(count):
                for c in range(nc):
                    v = flat[i * nc + c]
                    if v < mins[c]:
                        mins[c] = v
                    if v > maxs[c]:
                        maxs[c] = v
            acc["min"], acc["max"] = mins, maxs
        self.j["accessors"].append(acc)
        return len(self.j["accessors"]) - 1

    def add_sparse(self, total_count, idx_list, values_flat, type_="VEC3",
                   base_bv=None):
        """Morph-target style sparse float accessor.

        base_bv: optional zero-filled bufferView index used as the base
        (improves compatibility with loaders that ignore `sparse` or cannot
        handle accessors without a bufferView).
        """
        nc = _NCOMP[type_]
        iv = self.add_bv(struct.pack("<%dI" % len(idx_list), *idx_list))
        vv = self.add_bv(struct.pack("<%df" % len(values_flat), *values_flat))
        mins = [0.0] * nc
        maxs = [0.0] * nc
        for i in range(0, len(values_flat), nc):
            for c in range(nc):
                v = values_flat[i + c]
                if v < mins[c]:
                    mins[c] = v
                if v > maxs[c]:
                    maxs[c] = v
        acc = {"componentType": FLOAT, "count": total_count, "type": type_,
               "min": mins, "max": maxs,
               "sparse": {"count": len(idx_list),
                          "indices": {"bufferView": iv, "componentType": UINT},
                          "values": {"bufferView": vv}}}
        if base_bv is not None:
            acc["bufferView"] = base_bv
        self.j["accessors"].append(acc)
        return len(self.j["accessors"]) - 1

    def add_image(self, data: bytes, mime: str, name=None) -> int:
        bv = self.add_bv(data)
        img = {"bufferView": bv, "mimeType": mime}
        if name:
            img["name"] = name
        self.j.setdefault("images", []).append(img)
        return len(self.j["images"]) - 1

    # ---- output -----------------------------------------------------------
    def write_glb(self, path):
        while len(self.bin) % 4:
            self.bin.append(0)
        self.j["buffers"] = [{"byteLength": len(self.bin)}]
        js = json.dumps(self.j, ensure_ascii=False,
                        separators=(",", ":")).encode("utf-8")
        while len(js) % 4:
            js += b" "
        total = 12 + 8 + len(js) + 8 + len(self.bin)
        with open(path, "wb") as f:
            f.write(struct.pack("<4sII", b"glTF", 2, total))
            f.write(struct.pack("<I4s", len(js), b"JSON"))
            f.write(js)
            f.write(struct.pack("<I4s", len(self.bin), b"BIN\x00"))
            f.write(bytes(self.bin))
