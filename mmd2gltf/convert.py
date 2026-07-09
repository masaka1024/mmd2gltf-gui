# -*- coding: utf-8 -*-
"""PMX (+VMD) -> glTF 2.0 (.glb) conversion.

Coordinate conversion (MMD left-handed, Y-up  ->  glTF right-handed, Y-up):
    position / normal : (x, y, z) -> (x, y, -z)
    quaternion        : (x, y, z, w) -> (-x, -y, z, w)
    triangle winding  : (a, b, c) -> (a, c, b)
Everything glTF cannot express natively (IK, physics, sphere/toon shading,
material & bone morphs, SDEF, display frames, ...) is preserved verbatim
(MMD coordinate space) under `extras.mmd` in the glTF JSON.
"""
import io
import os
import struct
import sys

from .pmx import parse_pmx
from .vmd import parse_vmd
from . import gltf as G
from . import physics
from .bake_hair import bake_hair_into_gltf
from .animation import bake, Track, FPS

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def log(*a):
    print("[mmd2gltf]", *a, file=sys.stderr)


# Pillowが無いと read_texture() が中身をそのまま素通しできず None を返す形式。
# .png / .jpg はPillow無しでも生バイトのまま読めるためここには含めない。
_PIL_ONLY_EXTS = (".bmp", ".tga", ".sph", ".spa", ".dds", ".tif", ".tiff", ".gif")


def _warn_missing_pillow(model):
    """Pillow(PIL)が無いまま変換すると、.bmp/.tga/.sph/.spaなどのテクスチャが
    load_texture() 内で黙って None になり、材質からごっそり抜け落ちる
    (見た目は拡散色だけのベタ塗りになり「色が抜け落ちる」原因になる)。
    通常のWARNINGログは他の出力に埋もれて見逃しやすいため、対象テクスチャが
    実際にある場合だけ、変換の最初にまとめて目立つ形で警告する。
    """
    if HAS_PIL:
        return
    affected = [t for t in model["textures"]
                if os.path.splitext(t)[1].lower() in _PIL_ONLY_EXTS]
    if not affected:
        return
    log("=" * 60)
    log("警告: Pillow(PIL)が見つかりません。")
    log("次の %d 個のテクスチャは変換できず、材質から欠落します:"
        % len(affected))
    for t in affected[:10]:
        log("  -", t)
    if len(affected) > 10:
        log("  ... 他 %d 個" % (len(affected) - 10))
    log("修正方法: このプロジェクトのフォルダで次を実行してください:")
    log("  uv pip install pillow   (uvを使っていない場合は pip install Pillow)")
    log("インストール後、もう一度変換すると色が復元されます。")
    log("=" * 60)


# ---------------------------------------------------------------- coords
def cpos(v, s=1.0):
    return (v[0] * s, v[1] * s, -v[2] * s)


def cquat(q):
    return (-q[0], -q[1], q[2], q[3])


# ---------------------------------------------------------------- textures
def resolve_path(base_dir, rel):
    """Resolve a texture path case-insensitively (models authored on Windows)."""
    rel = rel.replace("\\", "/").strip()
    if not rel:
        return None
    p = os.path.normpath(os.path.join(base_dir, rel))
    if os.path.isfile(p):
        return p
    # case-insensitive walk
    cur = base_dir
    for part in os.path.normpath(rel).split(os.sep):
        if part in ("", "."):
            continue
        if part == "..":
            cur = os.path.dirname(cur)
            continue
        try:
            entries = os.listdir(cur)
        except OSError:
            return None
        match = next((e for e in entries if e.lower() == part.lower()), None)
        if match is None:
            return None
        cur = os.path.join(cur, match)
    return cur if os.path.isfile(cur) else None


def _alpha_class(img):
    """Classify a PIL image's alpha usage: 'opaque' | 'mask' | 'blend'.

    'mask'  = alpha is (almost) binary -> use alphaMode MASK (cutout).
              Typical for skin/face textures whose unused UV space is
              transparent; treating those as BLEND breaks depth sorting
              (faces look inside-out / see-through in many viewers).
    'blend' = significant fraction of semi-transparent texels
              (e.g. hair tips) -> real alpha blending.
    """
    if "A" not in img.getbands():
        # Palette ('P') images can carry transparency via a side-channel
        # (a 'transparency' tag / tRNS chunk) that does not show up as an
        # 'A' band in getbands(). Treating these as opaque silently drops
        # real transparency (e.g. cutout hair/eyelash textures authored as
        # indexed PNGs). Detect that case by converting to RGBA first.
        if img.mode == "P" and "transparency" in img.info:
            img = img.convert("RGBA")
        else:
            return "opaque"
    hist = img.getchannel("A").histogram()
    total = sum(hist) or 1
    below = sum(hist[:250]) / total
    if below == 0.0:
        return "opaque"
    # Only texels that are *really* see-through (alpha < 0.5) argue for
    # true blending.  Hair/skin textures are mostly high-alpha cutouts;
    # rendering them as BLEND disables depth writes in most viewers and
    # makes them look nearly transparent / wrongly sorted.
    semi_low = sum(hist[5:128]) / total
    return "blend" if semi_low > 0.25 else "mask"


def load_texture(path):
    """Return (png_or_jpg_bytes, mime, alpha_class) or None."""
    ext = os.path.splitext(path)[1].lower()
    if ext in (".jpg", ".jpeg") or (ext == ".png" and not HAS_PIL):
        with open(path, "rb") as f:
            data = f.read()
        mime = "image/png" if ext == ".png" else "image/jpeg"
        return data, mime, "opaque"
    if ext == ".png" and HAS_PIL:
        # re-encode to strip ICC/gamma chunks the glTF validator flags
        try:
            img = Image.open(path)
            aclass = _alpha_class(img)
            if img.mode not in ("RGB", "RGBA"):
                img = img.convert("RGB" if aclass == "opaque" else "RGBA")
            buf = io.BytesIO()
            img.save(buf, "PNG", icc_profile=None)
            return buf.getvalue(), "image/png", aclass
        except Exception as e:
            log("WARNING: failed to load texture %s (%s)" % (path, e))
            return None
    if not HAS_PIL:
        log("WARNING: Pillow not installed; cannot convert", path)
        return None
    try:
        img = Image.open(path)
        aclass = _alpha_class(img)
        if img.mode not in ("RGB", "RGBA"):
            img = img.convert("RGB" if aclass == "opaque" else "RGBA")
        buf = io.BytesIO()
        img.save(buf, "PNG")
        return buf.getvalue(), "image/png", aclass
    except Exception as e:
        log("WARNING: failed to load texture %s (%s)" % (path, e))
        return None


# ---------------------------------------------------------------- morphs
def collect_target_morphs(model, scale=1.0):
    """Decide which PMX morphs become glTF morph targets.

    vertex morphs -> POSITION deltas
    UV morphs     -> TEXCOORD_0 deltas
    group morphs  -> flattened, if every child is a vertex/UV morph
    Returns list of (morph_index, {'pos': {vidx: [dx,dy,dz]}, 'uv': {vidx: [du,dv]}}).

    `scale` is the global unit scale (see convert()); it is applied to the
    position deltas (a morph offset is a displacement in the same units as
    the vertex positions) but not to UV deltas.
    """
    morphs = model["morphs"]
    out = []

    def vertex_dict(mo, weight=1.0, acc=None):
        acc = acc if acc is not None else {"pos": {}, "uv": {}}
        t = mo["type"]
        if t == 1:
            for off in mo["offsets"]:
                d = acc["pos"].setdefault(off["vertex"], [0.0, 0.0, 0.0])
                for c in range(3):
                    d[c] += off["offset"][c] * weight * scale
        elif t == 3:
            for off in mo["offsets"]:
                d = acc["uv"].setdefault(off["vertex"], [0.0, 0.0])
                d[0] += off["offset"][0] * weight
                d[1] += off["offset"][1] * weight
        return acc

    for i, mo in enumerate(morphs):
        t = mo["type"]
        if t in (1, 3):
            out.append((i, vertex_dict(mo)))
        elif t == 0:
            children = [(o["morph"], o["weight"]) for o in mo["offsets"]]
            if all(0 <= ci < len(morphs) and morphs[ci]["type"] in (1, 3)
                   for ci, _ in children):
                acc = {"pos": {}, "uv": {}}
                for ci, w in children:
                    vertex_dict(morphs[ci], w, acc)
                out.append((i, acc))
    return out


# ---------------------------------------------------------------- main
def convert(pmx_path, out_path, vmd_path=None, unlit=False, solve_ik=True,
            step=1, extras=True, anim_name=None, disable_ik=None,
            use_vmd_ik_frames=True, morph_mode="sparse",
            alpha_mode="auto", force_double_sided=False,
            custom_attrs=True, scale=0.08,
            bake_physics=False, bake_target="hair",
            hair_drag=0.85, hair_stiffness=1.5, hair_gravity=0.02):
    """`scale` converts MMD units to glTF units (meters). PMX models are
    conventionally authored at roughly 1 MMD unit = 8cm (a ~160cm-tall
    character is about 20 units tall), but glTF assumes 1 unit = 1 meter, so
    without conversion a model renders ~12.5x too large in viewers that
    treat glTF units as meters. Default is 0.08; pass scale=1.0 to disable
    scaling if the source used a different convention. Applies to
    vertex/bone positions, SDEF params, morph position deltas, and baked
    animation translations. Normals, UVs, and rotations are unaffected.
    Data preserved verbatim under extras.mmd (rigid bodies, joints, etc.)
    stays in the original unscaled MMD units; the scale factor used is
    recorded there for reference.
    """
    model = parse_pmx(pmx_path)
    base_dir = os.path.dirname(os.path.abspath(pmx_path))
    log("PMX %.1f  '%s'  %d verts / %d tris / %d mats / %d bones / %d morphs"
        % (model["version"], model["name"], len(model["vertices"]),
           len(model["indices"]) // 3, len(model["materials"]),
           len(model["bones"]), len(model["morphs"])))

    _warn_missing_pillow(model)

    g = G.GltfBuilder()
    verts = model["vertices"]
    nv = len(verts)
    nb = len(model["bones"])
    if nb > 65535:
        raise ValueError("more than 65535 bones is not supported")

    # ---------------- vertex attributes ---------------------------------
    pos, nrm, uv = [], [], []
    joints, weights = [], []
    edge = []
    add_uv_n = model["add_uv_count"]
    add_uvs = [[] for _ in range(add_uv_n)]
    has_sdef = any(v["sdef"] for v in verts)
    sdef_c, sdef_r0, sdef_r1, wtype = [], [], [], []

    for v in verts:
        p = v["pos"]
        n = v["normal"]
        pos += [p[0] * scale, p[1] * scale, -p[2] * scale]
        nx, ny, nz = n[0], n[1], -n[2]
        nl = (nx * nx + ny * ny + nz * nz) ** 0.5
        if nl > 1e-6:
            nrm += [nx / nl, ny / nl, nz / nl]
        else:
            # MMDの一部頂点は法線が (0,0,0)（非表示・物理専用など）。
            # glTFは単位法線を要求するため、任意の単位ベクトルで補う。
            nrm += [0.0, 0.0, 1.0]
        uv += [v["uv"][0], v["uv"][1]]
        edge.append(v["edge"])
        for k in range(add_uv_n):
            add_uvs[k] += v["add_uv"][k]
        # skinning (SDEF/QDEF approximated as linear blend)
        # Merge duplicate bone indices before writing. MMD models often
        # author SDEF (and occasionally BDEF2) vertices with bone0 == bone1
        # -- common around ankles/elbows -- which would emit JOINTS_0 like
        # [9, 9, 0, 0]. The glTF validator rejects that
        # (ACCESSOR_JOINTS_INDEX_DUPLICATE). Summing the duplicate's weights
        # collapses it to one influence, then we normalize to 1.0.
        acc = {}
        for b, w in zip(v["bones"], v["weights"]):
            if w <= 0.0:
                continue
            b = b if 0 <= b < nb else 0
            acc[b] = acc.get(b, 0.0) + w
        if acc:
            top = sorted(acc.items(), key=lambda kv: -kv[1])[:4]
            bs = [b for b, _ in top]
            ws = [w for _, w in top]
            s = sum(ws)
            ws = [w / s for w in ws]
        else:
            bs, ws = [0], [1.0]
        while len(bs) < 4:
            bs.append(0)
            ws.append(0.0)
        joints += bs
        weights += ws
        if has_sdef:
            wtype.append(v["weight_type"])
            sd = v["sdef"]
            if sd:
                sdef_c += cpos(sd["c"], scale)
                sdef_r0 += cpos(sd["r0"], scale)
                sdef_r1 += cpos(sd["r1"], scale)
            else:
                sdef_c += [0.0, 0.0, 0.0]
                sdef_r0 += [0.0, 0.0, 0.0]
                sdef_r1 += [0.0, 0.0, 0.0]

    AB = G.ARRAY_BUFFER
    attrs = {
        "POSITION": g.add_accessor(pos, G.FLOAT, "VEC3", AB, minmax=True),
        "NORMAL": g.add_accessor(nrm, G.FLOAT, "VEC3", AB),
        "TEXCOORD_0": g.add_accessor(uv, G.FLOAT, "VEC2", AB),
        "JOINTS_0": g.add_accessor(joints, G.USHORT, "VEC4", AB),
        "WEIGHTS_0": g.add_accessor(weights, G.FLOAT, "VEC4", AB),
    }
    if custom_attrs:
        if any(e != 0.0 for e in edge):
            attrs["_EDGESCALE"] = g.add_accessor(edge, G.FLOAT, "SCALAR", AB)
        for k in range(add_uv_n):
            attrs["_ADDUV%d" % (k + 1)] = g.add_accessor(
                add_uvs[k], G.FLOAT, "VEC4", AB)
        if has_sdef:
            attrs["_SDEF_C"] = g.add_accessor(sdef_c, G.FLOAT, "VEC3", AB)
            attrs["_SDEF_R0"] = g.add_accessor(sdef_r0, G.FLOAT, "VEC3", AB)
            attrs["_SDEF_R1"] = g.add_accessor(sdef_r1, G.FLOAT, "VEC3", AB)
            # float, not ubyte: vertex attributes must be 4-byte aligned
            attrs["_WEIGHTTYPE"] = g.add_accessor([float(w) for w in wtype],
                                                  G.FLOAT, "SCALAR", AB)

    # ---------------- morph targets --------------------------------------
    # morph_mode:
    #   "sparse" - sparse accessors with a shared zero-filled base bufferView
    #              (small files; needs a loader with proper sparse support)
    #   "dense"  - plain full-size accessors (maximum compatibility)
    #   "none"   - no glTF morph targets (data still kept in extras)
    target_morphs = (collect_target_morphs(model, scale)
                     if morph_mode != "none" else [])
    morph_to_target = {mi: ti for ti, (mi, _) in enumerate(target_morphs)}
    targets = []
    target_names = []
    zero_bv3 = zero_bv2 = None
    if target_morphs and morph_mode == "sparse":
        # shared zero-filled bases; byteStride + target required by the
        # validator when several vertex-attribute accessors share one view
        zero_bv3 = g.add_bv(bytes(nv * 12), target=AB, byte_stride=12)
        if any(d["uv"] for _, d in target_morphs):
            zero_bv2 = g.add_bv(bytes(nv * 8), target=AB, byte_stride=8)
    for mi, deltas in target_morphs:
        tgt = {}
        pos_d = deltas["pos"]
        if morph_mode == "dense":
            flat = [0.0] * (nv * 3)
            for vi, d in pos_d.items():
                flat[vi * 3] = d[0]
                flat[vi * 3 + 1] = d[1]
                flat[vi * 3 + 2] = -d[2]
            tgt["POSITION"] = g.add_accessor(flat, G.FLOAT, "VEC3",
                                             minmax=True)
            if deltas["uv"]:
                flat = [0.0] * (nv * 2)
                for vi, d in deltas["uv"].items():
                    flat[vi * 2] = d[0]
                    flat[vi * 2 + 1] = d[1]
                tgt["TEXCOORD_0"] = g.add_accessor(flat, G.FLOAT, "VEC2")
        else:
            if pos_d:
                idxs = sorted(pos_d)
                flat = []
                for vi in idxs:
                    d = pos_d[vi]
                    flat += [d[0], d[1], -d[2]]
                tgt["POSITION"] = g.add_sparse(nv, idxs, flat, "VEC3",
                                               base_bv=zero_bv3)
            else:
                tgt["POSITION"] = g.add_sparse(nv, [0], [0.0, 0.0, 0.0],
                                               "VEC3", base_bv=zero_bv3)
            uv_d = deltas["uv"]
            if uv_d:
                idxs = sorted(uv_d)
                flat = []
                for vi in idxs:
                    flat += uv_d[vi]
                tgt["TEXCOORD_0"] = g.add_sparse(nv, idxs, flat, "VEC2",
                                                 base_bv=zero_bv2)
        targets.append(tgt)
        target_names.append(model["morphs"][mi]["name"])

    # ---------------- textures -------------------------------------------
    tex_map = {}   # pmx texture index -> gltf texture index
    tex_alpha = {}
    if model["textures"]:
        g.j["samplers"] = [{"magFilter": 9729, "minFilter": 9987,
                            "wrapS": 10497, "wrapT": 10497}]
        g.j["textures"] = []
    for ti, rel in enumerate(model["textures"]):
        p = resolve_path(base_dir, rel)
        if p is None:
            log("WARNING: texture not found:", rel)
            continue
        res = load_texture(p)
        if res is None:
            continue
        data, mime, aclass = res
        img = g.add_image(data, mime, name=os.path.basename(rel))
        g.j["textures"].append({"sampler": 0, "source": img,
                                "name": os.path.basename(rel)})
        tex_map[ti] = len(g.j["textures"]) - 1
        tex_alpha[ti] = aclass

    # ---------------- materials -------------------------------------------
    g.j["materials"] = []
    for m in model["materials"]:
        mat = {
            "name": m["name"] or m["name_en"],
            "pbrMetallicRoughness": {
                "baseColorFactor": [max(0.0, min(1.0, c)) for c in m["diffuse"]],
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0,
            },
            "doubleSided": force_double_sided or bool(m["flags"] & 0x01),
        }
        ti = m["texture"]
        aclass = "opaque"
        if ti in tex_map:
            mat["pbrMetallicRoughness"]["baseColorTexture"] = {
                "index": tex_map[ti]}
            aclass = tex_alpha.get(ti, "opaque")
        if alpha_mode != "auto":
            mode = alpha_mode.upper()
        elif m["diffuse"][3] < 1.0 or aclass == "blend":
            mode = "BLEND"
        elif aclass == "mask":
            mode = "MASK"
        else:
            mode = "OPAQUE"
        mat["alphaMode"] = mode
        if mode == "MASK":
            mat["alphaCutoff"] = 0.5
        if unlit:
            mat["extensions"] = {"KHR_materials_unlit": {}}
        if extras:
            mat["extras"] = {"mmd": {
                "nameEn": m["name_en"],
                "ambient": m["ambient"],
                "specular": m["specular"],
                "specularPower": m["specular_power"],
                "flags": m["flags"],
                "edgeColor": m["edge_color"],
                "edgeSize": m["edge_size"],
                "sphereMode": m["sphere_mode"],
                "sphereTexture": tex_map.get(m["sphere_texture"], -1),
                "toonTexture": tex_map.get(m["toon_texture"], -1),
                "toonShared": m["toon_shared"],
                "memo": m["memo"],
            }}
        g.j["materials"].append(mat)
    if unlit:
        g.j["extensionsUsed"] = ["KHR_materials_unlit"]

    # ---------------- mesh primitives --------------------------------------
    idx = model["indices"]
    prims = []
    offset = 0
    for mi, m in enumerate(model["materials"]):
        cnt = m["index_count"]
        chunk = idx[offset:offset + cnt]
        offset += cnt
        if not chunk:
            continue
        flipped = []
        for t in range(0, len(chunk) - 2, 3):
            flipped += [chunk[t], chunk[t + 2], chunk[t + 1]]
        acc = g.add_accessor(flipped, G.UINT, "SCALAR",
                             G.ELEMENT_ARRAY_BUFFER)
        prim = {"attributes": attrs, "indices": acc, "material": mi,
                "mode": 4}
        if targets:
            prim["targets"] = targets
        prims.append(prim)

    mesh = {"name": model["name"] or "mesh", "primitives": prims}
    if targets:
        mesh["weights"] = [0.0] * len(targets)
        mesh["extras"] = {"targetNames": target_names}
    g.j["meshes"] = [mesh]

    # ---------------- nodes / skeleton -------------------------------------
    bones = model["bones"]
    nodes = []
    world = []
    for i, b in enumerate(bones):
        p = b["parent"]
        wp = tuple(b["pos"])
        world.append(wp)
        if 0 <= p < nb:
            lt = (wp[0] - bones[p]["pos"][0], wp[1] - bones[p]["pos"][1],
                  wp[2] - bones[p]["pos"][2])
        else:
            lt = wp
        node = {"name": b["name"] or ("bone_%d" % i),
                "translation": list(cpos(lt, scale))}
        if extras:
            ex = {"nameEn": b["name_en"], "flags": b["flags"],
                  "layer": b["layer"]}
            for key in ("tail_bone", "tail_offset", "inherit_parent",
                        "inherit_ratio", "fixed_axis", "local_x", "local_z",
                        "external_key", "ik"):
                if key in b:
                    ex[key] = b[key]
            node["extras"] = {"mmd": ex}
        nodes.append(node)
    root_bones = []
    for i, b in enumerate(bones):
        p = b["parent"]
        if 0 <= p < nb:
            nodes[p].setdefault("children", []).append(i)
        else:
            root_bones.append(i)

    ibm = []
    for wp in world:
        w = cpos(wp, scale)
        ibm += [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -w[0], -w[1], -w[2], 1]
    ibm_acc = g.add_accessor(ibm, G.FLOAT, "MAT4")

    # skinned mesh node must be a scene root (its transform is ignored)
    mesh_node = len(nodes)
    nodes.append({"name": model["name"] or "model", "mesh": 0, "skin": 0})
    root = len(nodes)
    nodes.append({"name": (model["name"] or "root") + "_skeleton",
                  "children": root_bones})
    g.j["nodes"] = nodes
    g.j["skins"] = [{"joints": list(range(nb)),
                     "inverseBindMatrices": ibm_acc,
                     "skeleton": root}]
    g.j["scenes"] = [{"nodes": [root, mesh_node]}]
    g.j["scene"] = 0

    # ---------------- MMD extras --------------------------------------------
    if extras:
        morph_extras = []
        for i, mo in enumerate(model["morphs"]):
            me = {"name": mo["name"], "nameEn": mo["name_en"],
                  "panel": mo["panel"], "type": mo["type"],
                  "target": morph_to_target.get(i)}
            if mo["type"] not in (1, 3):
                me["offsets"] = mo["offsets"]
            morph_extras.append(me)
        g.j["extras"] = {"mmd": {
            "coordinateNote": "values under extras.mmd are raw PMX values in "
                              "MMD left-handed space (x,y,z), unscaled "
                              "(see unitScale); glTF space is (x,y,-z), "
                              "quat (-x,-y,z,w)",
            "unitScale": scale,
            "format": "pmx",
            "version": model["version"],
            "name": model["name"], "nameEn": model["name_en"],
            "comment": model["comment"], "commentEn": model["comment_en"],
            "morphs": morph_extras,
            "displayFrames": model["display_frames"],
            "rigidBodies": model["rigid_bodies"],
            "joints": model["joints"],
        }}

        # 変換済み物理ビュー（後工程向け・ボーンローカル）を併載。
        # raw の rigidBodies/joints は保持したまま physicsGltf を追加する。
        _phys = g.j["extras"]["mmd"]
        if _phys.get("rigidBodies"):
            _bwm = physics.compute_bone_world_matrices(g.j)
            _phys["physicsGltf"] = physics.build_physics_gltf(_phys, _bwm)

    # ---------------- animation ---------------------------------------------
    if vmd_path:
        vmd = parse_vmd(vmd_path)
        log("VMD '%s'  %d bone tracks / %d morph tracks / last frame %d"
            % (vmd["model_name"], len(vmd["bones"]), len(vmd["morphs"]),
               vmd["max_frame"]))
        anim = {"name": anim_name or os.path.splitext(
            os.path.basename(vmd_path))[0], "samplers": [], "channels": []}

        if vmd["bones"]:
            def prog(i, n):
                log("  baking frame %d/%d" % (i, n))
            if disable_ik:
                log("  IK disabled for bones matching:", disable_ik)
            if vmd.get("ik_frames"):
                log("  VMD contains %d IK on/off key frame(s)%s"
                    % (len(vmd["ik_frames"]),
                       "" if use_vmd_ik_frames else " (ignored)"))
            times, baked, unmatched = bake(
                model, vmd, solve_ik=solve_ik, step=step, progress=prog,
                disable_ik=disable_ik, use_vmd_ik_frames=use_vmd_ik_frames)
            if unmatched:
                log("  %d VMD bone tracks had no matching PMX bone"
                    % len(unmatched))
            t_acc = g.add_accessor(times, G.FLOAT, "SCALAR", minmax=True)

            # --- 剛体物理ベイク（PBD/クロス）: 剛体の回転を物理シミュで生成 ---
            # bake_target: "hair"=髪のみ / "all"=全dynamic剛体
            #   （スカート等のリング構造もクロスシミュで扱う）
            hair_keys = {}
            if bake_physics:
                _phys = g.j.get("extras", {}).get("mmd", {}).get("physicsGltf")
                if _phys is None:
                    _mmdx = {"unitScale": scale,
                             "rigidBodies": model["rigid_bodies"],
                             "joints": model["joints"]}
                    _phys = physics.build_physics_gltf(
                        _mmdx, physics.compute_bone_world_matrices(g.j))
                if _phys.get("rigidBodies"):
                    _only = ["髪"] if bake_target == "hair" else None
                    hair_keys, _n_excl = bake_hair_into_gltf(
                        g.j, baked, len(times), _phys, scale,
                        drag_force=hair_drag, stiffness_force=hair_stiffness,
                        gravity_power=hair_gravity, fps=FPS, only_names=_only)
                    log("  physics baked: %d bone(s) (%s)"
                        % (len(hair_keys),
                           "hair only" if bake_target == "hair" else "all"))
                    if _n_excl:
                        log("  skipped %d unreachable rigid bodies" % _n_excl)
            hair_bones = set(hair_keys)
            for bi, data in sorted(baked.items()):
                if data["r"] and bi not in hair_bones:
                    flat = []
                    prev = None
                    for q in data["r"]:
                        q = cquat(q)
                        if prev and (q[0] * prev[0] + q[1] * prev[1] +
                                     q[2] * prev[2] + q[3] * prev[3]) < 0:
                            q = (-q[0], -q[1], -q[2], -q[3])
                        flat += q
                        prev = q
                    out = g.add_accessor(flat, G.FLOAT, "VEC4")
                    anim["samplers"].append({"input": t_acc, "output": out,
                                             "interpolation": "LINEAR"})
                    anim["channels"].append({
                        "sampler": len(anim["samplers"]) - 1,
                        "target": {"node": bi, "path": "rotation"}})
                if data["t"]:
                    flat = []
                    for t in data["t"]:
                        flat += cpos(t, scale)
                    out = g.add_accessor(flat, G.FLOAT, "VEC3")
                    anim["samplers"].append({"input": t_acc, "output": out,
                                             "interpolation": "LINEAR"})
                    anim["channels"].append({
                        "sampler": len(anim["samplers"]) - 1,
                        "target": {"node": bi, "path": "translation"}})
            for bi, qs in sorted(hair_keys.items()):
                flat = []
                prev = None
                for q in qs:                       # 既にglTF局所回転（cquat不要）
                    if prev and (q[0] * prev[0] + q[1] * prev[1] +
                                 q[2] * prev[2] + q[3] * prev[3]) < 0:
                        q = (-q[0], -q[1], -q[2], -q[3])
                    flat += list(q)
                    prev = q
                out = g.add_accessor(flat, G.FLOAT, "VEC4")
                anim["samplers"].append({"input": t_acc, "output": out,
                                         "interpolation": "LINEAR"})
                anim["channels"].append({
                    "sampler": len(anim["samplers"]) - 1,
                    "target": {"node": bi, "path": "rotation"}})
            if hair_keys:
                log("  physics rotation channels: %d" % len(hair_keys))
            log("  bone animation: %d channels, %d key frames"
                % (len(anim["channels"]), len(times)))

        # morph (weights) animation -- linear between keys
        if vmd["morphs"] and targets:
            tmap = {}   # target index -> key list
            skipped = 0
            for i, (mi, _) in enumerate(target_morphs):
                name = model["morphs"][mi]["name"]
                key = name.encode("shift-jis", errors="replace")[:15]
                found = None
                for vname, keys in vmd["morphs"].items():
                    if vname.encode("shift-jis", errors="replace")[:15] == key:
                        found = keys
                        break
                if found:
                    tmap[i] = found
            for vname in vmd["morphs"]:
                if not any(model["morphs"][mi]["name"]
                           .encode("shift-jis", errors="replace")[:15] ==
                           vname.encode("shift-jis", errors="replace")[:15]
                           for mi, _ in target_morphs):
                    skipped += 1
            if skipped:
                log("  %d VMD morph tracks target non-vertex/UV morphs "
                    "(kept in extras only)" % skipped)
            if tmap:
                frames = sorted({f for keys in tmap.values()
                                 for f, _ in keys})
                w_times = [f / FPS for f in frames]
                flat = []
                for f in frames:
                    for ti in range(len(targets)):
                        keys = tmap.get(ti)
                        if not keys:
                            flat.append(0.0)
                            continue
                        # linear evaluation
                        if f <= keys[0][0]:
                            flat.append(keys[0][1])
                        elif f >= keys[-1][0]:
                            flat.append(keys[-1][1])
                        else:
                            for k in range(len(keys) - 1):
                                f0, w0 = keys[k]
                                f1, w1 = keys[k + 1]
                                if f0 <= f <= f1:
                                    r = (f - f0) / (f1 - f0) if f1 > f0 else 0
                                    flat.append(w0 + (w1 - w0) * r)
                                    break
                in_acc = g.add_accessor(w_times, G.FLOAT, "SCALAR",
                                        minmax=True)
                out_acc = g.add_accessor(flat, G.FLOAT, "SCALAR")
                anim["samplers"].append({"input": in_acc, "output": out_acc,
                                         "interpolation": "LINEAR"})
                anim["channels"].append({
                    "sampler": len(anim["samplers"]) - 1,
                    "target": {"node": mesh_node, "path": "weights"}})
                log("  morph animation: %d tracks, %d key frames"
                    % (len(tmap), len(frames)))

        if anim["channels"]:
            g.j["animations"] = [anim]

    g.write_glb(out_path)
    log("wrote", out_path, "(%.1f MB)" % (os.path.getsize(out_path) / 1e6))
    return out_path
