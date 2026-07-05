# -*- coding: utf-8 -*-
"""Generate a small synthetic PMX 2.0 model + VMD motion for testing.

Model: a 2-segment vertical "arm" (8 vertices, 2 boxes-ish quads), 4 bones
(センター -> 上, 上2 [IK chain], IK bone), 1 textured material, vertex morph,
UV morph, group morph, bone morph, material morph, rigid body, joint,
SDEF vertex, addUV.
"""
import io
import math
import os
import struct

OUT = os.path.dirname(os.path.abspath(__file__))


def txt(s):
    b = s.encode("utf-16-le")
    return struct.pack("<i", len(b)) + b


def f(*v):
    return struct.pack("<%df" % len(v), *v)


def make_pmx(path):
    o = io.BytesIO()
    o.write(b"PMX ")
    o.write(struct.pack("<f", 2.0))
    # globals: utf16, 1 addUV, vert idx 2, tex 1, mat 1, bone 1, morph 1, rigid 1
    o.write(struct.pack("<B8B", 8, 0, 1, 2, 1, 1, 1, 1, 1))
    o.write(txt("テストモデル") + txt("TestModel"))
    o.write(txt("コメント") + txt("comment"))

    # vertices: 8 (two quads stacked), various weight types
    verts = []
    for i in range(8):
        y = (i // 4) * 5.0 + (i % 4 // 2) * 5.0
        x = -1.0 if i % 2 == 0 else 1.0
        z = 0.0
        verts.append((x, y, z))
    o.write(struct.pack("<i", 8))
    for i, (x, y, z) in enumerate(verts):
        o.write(f(x, y, z))          # pos
        o.write(f(0, 0, -1))         # normal
        o.write(f((x + 1) / 2, 1 - y / 10))  # uv
        o.write(f(0.1, 0.2, 0.3, 0.4))       # addUV1
        if i == 0:      # BDEF1
            o.write(struct.pack("<Bb", 0, 1))
        elif i == 1:    # BDEF2
            o.write(struct.pack("<Bbb", 1, 1, 2) + f(0.7))
        elif i == 2:    # SDEF
            o.write(struct.pack("<Bbb", 3, 1, 2) + f(0.5))
            o.write(f(0, 5, 0) + f(0, 4, 0) + f(0, 6, 0))
        elif i == 3:    # BDEF4
            o.write(struct.pack("<Bbbbb", 2, 1, 2, 0, 0) + f(0.5, 0.5, 0, 0))
        else:
            w = 1.0 if y > 7 else 0.5
            o.write(struct.pack("<Bbb", 1, 2, 1) + f(w))
        o.write(f(1.0))              # edge scale

    # faces: 4 triangles (two quads)
    idx = [0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7]
    o.write(struct.pack("<i", len(idx)))
    o.write(struct.pack("<%dH" % len(idx), *idx))

    # textures
    o.write(struct.pack("<i", 1))
    o.write(txt("tex.png"))

    # materials: 2 (first 2 tris, last 2 tris)
    o.write(struct.pack("<i", 2))
    for mi in range(2):
        o.write(txt("材質%d" % mi) + txt("mat%d" % mi))
        o.write(f(1, 1, 1, 1 if mi == 0 else 0.5))   # diffuse
        o.write(f(0.3, 0.3, 0.3) + f(5.0))           # specular, power
        o.write(f(0.5, 0.5, 0.5))                    # ambient
        o.write(struct.pack("<B", 0x01))             # double-sided
        o.write(f(0, 0, 0, 1) + f(1.0))              # edge color, size
        o.write(struct.pack("<bb", 0 if mi == 0 else -1, -1))  # tex, sphere
        o.write(struct.pack("<B", 0))                # sphere mode
        o.write(struct.pack("<Bb", 1, 1))            # shared toon 1
        o.write(txt(""))
        o.write(struct.pack("<i", 6))

    # bones: 0 センター(root), 1 上(child), 2 上2(child), 3 IK
    o.write(struct.pack("<i", 4))
    # bone 0
    o.write(txt("センター") + txt("center"))
    o.write(f(0, 0, 0))
    o.write(struct.pack("<b", -1) + struct.pack("<i", 0))
    o.write(struct.pack("<H", 0x0002 | 0x0004 | 0x0008 | 0x0010))
    o.write(f(0, 5, 0))    # tail offset
    # bone 1
    o.write(txt("上") + txt("upper"))
    o.write(f(0, 5, 0))
    o.write(struct.pack("<b", 0) + struct.pack("<i", 0))
    o.write(struct.pack("<H", 0x0001 | 0x0002 | 0x0008 | 0x0010))
    o.write(struct.pack("<b", 2))   # tail bone
    # bone 2
    o.write(txt("上2") + txt("upper2"))
    o.write(f(0, 10, 0))
    o.write(struct.pack("<b", 1) + struct.pack("<i", 0))
    o.write(struct.pack("<H", 0x0002 | 0x0008 | 0x0010))
    o.write(f(0, 2, 0))
    # bone 3: IK bone targeting bone2, link bone1 (with limits)
    o.write(txt("ＩＫ") + txt("ik"))
    o.write(f(0, 10, 0))
    o.write(struct.pack("<b", -1) + struct.pack("<i", 0))
    o.write(struct.pack("<H", 0x0002 | 0x0004 | 0x0008 | 0x0010 | 0x0020))
    o.write(f(0, 0, 0))          # tail offset
    o.write(struct.pack("<b", 2))                 # ik target
    o.write(struct.pack("<i", 10) + f(math.radians(57)))
    o.write(struct.pack("<i", 1))
    o.write(struct.pack("<bB", 1, 1))             # link bone1, limited
    o.write(f(-math.pi, 0, 0) + f(0, 0, 0))

    # morphs: vertex, UV, group, bone, material
    o.write(struct.pack("<i", 5))
    o.write(txt("にこ") + txt("smile") + struct.pack("<BBi", 1, 1, 2))
    o.write(struct.pack("<H", 0) + f(0.5, 0, -0.3))
    o.write(struct.pack("<H", 1) + f(-0.5, 0, -0.3))
    o.write(txt("UVモーフ") + txt("uvm") + struct.pack("<BBi", 1, 3, 1))
    o.write(struct.pack("<H", 0) + f(0.1, -0.1, 0, 0))
    o.write(txt("グループ") + txt("grp") + struct.pack("<BBi", 4, 0, 2))
    o.write(struct.pack("<b", 0) + f(0.5))
    o.write(struct.pack("<b", 1) + f(1.0))
    o.write(txt("ボーンモーフ") + txt("bonem") + struct.pack("<BBi", 4, 2, 1))
    o.write(struct.pack("<b", 1) + f(0, 1, 0) + f(0, 0, 0, 1))
    o.write(txt("材質モーフ") + txt("matm") + struct.pack("<BBi", 4, 8, 1))
    o.write(struct.pack("<bB", 0, 0))
    o.write(f(1, 0, 0, 1) + f(0, 0, 0) + f(1) + f(0, 0, 0))
    o.write(f(0, 0, 0, 1) + f(0))
    o.write(f(1, 1, 1, 1) + f(1, 1, 1, 1) + f(1, 1, 1, 1))

    # display frames
    o.write(struct.pack("<i", 2))
    o.write(txt("Root") + txt("Root") + struct.pack("<B", 1))
    o.write(struct.pack("<i", 1) + struct.pack("<Bb", 0, 0))
    o.write(txt("表情") + txt("Exp") + struct.pack("<B", 1))
    o.write(struct.pack("<i", 1) + struct.pack("<Bb", 1, 0))

    # rigid bodies
    o.write(struct.pack("<i", 2))
    for ri in range(2):
        o.write(txt("剛体%d" % ri) + txt("rb%d" % ri))
        o.write(struct.pack("<b", ri + 1))
        o.write(struct.pack("<BH", ri, 0xFFFF))
        o.write(struct.pack("<B", 0))          # sphere
        o.write(f(1, 0, 0))
        o.write(f(0, 5 * (ri + 1), 0) + f(0, 0, 0))
        o.write(f(1.0, 0.5, 0.5, 0.0, 0.5))
        o.write(struct.pack("<B", ri))

    # joints
    o.write(struct.pack("<i", 1))
    o.write(txt("ジョイント") + txt("joint0"))
    o.write(struct.pack("<B", 0))
    o.write(struct.pack("<bb", 0, 1))
    o.write(f(0, 7.5, 0) + f(0, 0, 0))
    o.write(f(0, 0, 0) + f(0, 0, 0))
    o.write(f(-0.1, -0.1, -0.1) + f(0.1, 0.1, 0.1))
    o.write(f(0, 0, 0) + f(0, 0, 0))

    with open(path, "wb") as fp:
        fp.write(o.getvalue())


def make_vmd(path):
    o = io.BytesIO()
    o.write(b"Vocaloid Motion Data 0002".ljust(30, b"\x00"))
    o.write("テストモデル".encode("shift-jis").ljust(20, b"\x00"))

    def bone_frame(name, frame, pos, rot, curves=None):
        o.write(name.encode("shift-jis").ljust(15, b"\x00"))
        o.write(struct.pack("<I", frame))
        o.write(struct.pack("<7f", *pos, *rot))
        interp = bytearray(64)
        c = curves or (20, 20, 107, 107)
        for i in range(4):
            interp[i] = c[0]
            interp[i + 4] = c[1]
            interp[i + 8] = c[2]
            interp[i + 12] = c[3]
        o.write(bytes(interp))

    s45 = math.sin(math.radians(22.5))
    c45 = math.cos(math.radians(22.5))
    frames = [
        ("センター", 0, (0, 0, 0), (0, 0, 0, 1), None),
        ("センター", 30, (0, 1, 2), (0, 0, 0, 1), (60, 10, 70, 110)),
        ("上", 0, (0, 0, 0), (0, 0, 0, 1), None),
        ("上", 30, (0, 0, 0), (s45, 0, 0, c45), None),
        ("ＩＫ", 0, (0, 0, 0), (0, 0, 0, 1), None),
        ("ＩＫ", 30, (0, -3, -2), (0, 0, 0, 1), None),
    ]
    o.write(struct.pack("<I", len(frames)))
    for fr in frames:
        bone_frame(*fr)

    morphs = [("にこ", 0, 0.0), ("にこ", 20, 1.0), ("にこ", 40, 0.0),
              ("グループ", 10, 0.5)]
    o.write(struct.pack("<I", len(morphs)))
    for name, frame, w in morphs:
        o.write(name.encode("shift-jis").ljust(15, b"\x00"))
        o.write(struct.pack("<If", frame, w))

    # empty camera / light / shadow sections
    o.write(struct.pack("<I", 0))
    o.write(struct.pack("<I", 0))
    o.write(struct.pack("<I", 0))

    with open(path, "wb") as fp:
        fp.write(o.getvalue())


def make_texture(path):
    try:
        from PIL import Image
        img = Image.new("RGBA", (32, 32))
        for y in range(32):
            for x in range(32):
                img.putpixel((x, y), (x * 8, y * 8, 128, 255))
        img.save(path)
    except ImportError:
        # 1x1 PNG fallback
        import base64
        png = base64.b64decode(
            b"iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR4nGNg"
            b"YGD4DwABBAEAX+XLSgAAAABJRU5ErkJggg==")
        with open(path, "wb") as fp:
            fp.write(png)


if __name__ == "__main__":
    make_pmx(os.path.join(OUT, "test.pmx"))
    make_vmd(os.path.join(OUT, "test.vmd"))
    make_texture(os.path.join(OUT, "tex.png"))
    print("test data written to", OUT)
