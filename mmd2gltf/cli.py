# -*- coding: utf-8 -*-
import argparse
import os
import sys

from .convert import convert


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="mmd2gltf",
        description="Convert MMD PMX models (and optional VMD motions) "
                    "to glTF 2.0 (.glb) as faithfully as possible.")
    ap.add_argument("pmx", help="input .pmx model")
    ap.add_argument("-o", "--output", help="output .glb path "
                    "(default: <pmx name>.glb)")
    ap.add_argument("--vmd", help="optional .vmd motion to bake as a "
                    "glTF animation")
    ap.add_argument("--no-ik", action="store_true",
                    help="do not solve IK while baking the motion")
    ap.add_argument("--disable-ik", action="append", metavar="NAME",
                    help="disable IK bones whose name contains NAME "
                    "(repeatable; e.g. --disable-ik 足 for leg/toe IK)")
    ap.add_argument("--ignore-vmd-ik", action="store_true",
                    help="ignore IK on/off key frames stored in the VMD")
    ap.add_argument("--step", type=int, default=1, metavar="N",
                    help="bake every N-th VMD frame (default 1 = 30fps)")
    ap.add_argument("--unlit", action="store_true",
                    help="tag materials with KHR_materials_unlit "
                    "(closer to MMD toon look in many viewers)")
    ap.add_argument("--no-extras", action="store_true",
                    help="omit extras.mmd metadata (IK/physics/morph data)")
    ap.add_argument("--anim-name", help="name for the glTF animation")
    ap.add_argument("--alpha-mode", choices=["auto", "opaque", "mask", "blend"],
                    default="auto",
                    help="alphaMode for all materials; auto analyses each "
                    "texture's alpha channel (default: auto)")
    ap.add_argument("--no-custom-attrs", action="store_true",
                    help="omit MMD-specific vertex attributes (_SDEF_*, "
                    "_ADDUV*, _EDGESCALE, _WEIGHTTYPE); try this if an "
                    "importer (e.g. Blender) fails on custom attributes")
    ap.add_argument("--force-double-sided", action="store_true",
                    help="render all materials double-sided "
                    "(like three.js MMDLoader)")
    ap.add_argument("--morph-mode", choices=["sparse", "dense", "none"],
                    default="sparse",
                    help="morph target encoding: sparse=small files, "
                    "dense=maximum viewer compatibility, none=no targets "
                    "(default: sparse)")
    ap.add_argument("--scale", type=float, default=0.08, metavar="F",
                    help="uniform scale from MMD units to glTF units "
                    "(meters). MMD models are conventionally ~1 unit=8cm, "
                    "so the default of 0.08 fixes models that would "
                    "otherwise appear ~12.5x too large in glTF viewers "
                    "that treat glTF units as meters. Pass --scale 1.0 to "
                    "disable scaling (default: 0.08)")
    ap.add_argument("--bake-physics", action="store_true",
                    help="bake rigid-body physics into bone keyframes (PBD "
                    "spring sim); fixes frozen/IK-locked hair, ties, etc.")
    ap.add_argument("--bake-target", choices=["hair", "all"], default="hair",
                    help="which rigid bodies to bake: hair=hair only, "
                    "all=all dynamic bodies incl. skirts (cloth solver "
                    "handles ring structures) (default: hair)")
    ap.add_argument("--hair-drag", type=float, default=0.85, metavar="F",
                    help="velocity retention 0..1 (default 0.85; higher = "
                    "floppier)")
    ap.add_argument("--hair-stiffness", type=float, default=1.5, metavar="F",
                    help="rest-shape restoring force (default 1.5)")
    ap.add_argument("--hair-gravity", type=float, default=0.02, metavar="F",
                    help="gravity strength (default 0.02; 0 keeps rest shape)")
    ap.add_argument("--collision-margin", type=float, default=0.01, metavar="F",
                    help="clearance kept between cloth and body colliders in "
                    "glTF units (default 0.01). Increase if the skirt visually "
                    "touches/clips the legs; 0 = push to the collider surface "
                    "exactly")
    a = ap.parse_args(argv)

    out = a.output or os.path.splitext(a.pmx)[0] + ".glb"
    try:
        convert(a.pmx, out, vmd_path=a.vmd, unlit=a.unlit,
                solve_ik=not a.no_ik, step=max(1, a.step),
                extras=not a.no_extras, anim_name=a.anim_name,
                disable_ik=a.disable_ik,
                use_vmd_ik_frames=not a.ignore_vmd_ik,
                morph_mode=a.morph_mode, alpha_mode=a.alpha_mode,
                force_double_sided=a.force_double_sided,
                custom_attrs=not a.no_custom_attrs, scale=a.scale,
                bake_physics=a.bake_physics, bake_target=a.bake_target,
                hair_drag=a.hair_drag, hair_stiffness=a.hair_stiffness,
                hair_gravity=a.hair_gravity,
                collision_margin=a.collision_margin)
    except Exception as e:
        print("error:", e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
