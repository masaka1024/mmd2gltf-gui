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
    ap.add_argument("--force-no-collision", action="append", metavar="NAME",
                    help="rigid body name to force fully non-colliding, "
                    "overriding its PMX group/noCollisionMask (denylist "
                    "escape hatch; repeatable). Use when a specific collider "
                    "pins/stretches cloth in extreme poses despite the PMX "
                    "data appearing correctly configured")
    ap.add_argument("--allowed-collider", action="append", metavar="NAME",
                    help="switch collision to allowlist mode: only the named "
                    "rigid bodies are treated as colliders at all (repeatable; "
                    "ignores PMX group/noCollisionMask entirely once any "
                    "--allowed-collider is given). Mirrors VRM SpringBone / "
                    "VRChat PhysBones-style per-chain collider scoping")
    ap.add_argument("--hem-extra-margin", type=float, default=0.0, metavar="F",
                    help="extra clearance added on top of --collision-margin, "
                    "applied only to the last (hem/tip) particle of each skirt "
                    "chain (default 0.0 = off). Unlike raising --collision-margin "
                    "globally, this does not push waist-side segments outward "
                    "(verified: waist angle stays fixed while raising this)")
    ap.add_argument("--adaptive-substep-threshold", type=float, default=None,
                    metavar="F",
                    help="enable adaptive substepping for cloth (skirts etc.) "
                    "to reduce bone-level tunneling through colliders during "
                    "fast motion. A frame is subdivided when the estimated "
                    "anchor+collider relative movement exceeds this fraction "
                    "of the nearest collider's radius (e.g. 0.5). Default "
                    "None = disabled (bit-identical to not having this "
                    "feature). Typical values from measurement: 0.5-0.75")
    ap.add_argument("--adaptive-substep-max-n", type=int, default=4, metavar="N",
                    help="cap on how many substeps a single frame can be "
                    "split into (default 4)")
    ap.add_argument("--adaptive-substep-collider", action="append",
                    metavar="NAME",
                    help="rigid body name to consider when deciding whether "
                    "to trigger adaptive substepping (repeatable; e.g. pass "
                    "leg/knee/thigh collider names to ignore fast arm motion). "
                    "Default: consider all colliders used for collision")
    ap.add_argument("--midpoint-correction", action="store_true",
                    help="enable an extra pass that nudges real bone "
                    "positions apart when the midpoint of the mesh segment "
                    "between two adjacent skirt bones is found inside a "
                    "collider (mitigates visible mesh-through-leg crossing "
                    "that bone-level collision alone cannot detect). "
                    "Measured to cut penetration depth/area by roughly "
                    "60-75%%, but does not eliminate it entirely")
    ap.add_argument("--midpoint-correction-iters", type=int, default=2,
                    metavar="N",
                    help="relaxation iterations for --midpoint-correction "
                    "(default 2; cost overhead is small even at 6)")
    ap.add_argument("--midpoint-correction-margin", type=float, default=0.0,
                    metavar="F",
                    help="clearance used by --midpoint-correction (default 0.0)")
    ap.add_argument("--midpoint-correction-collider", action="append",
                    metavar="NAME",
                    help="rigid body name to consider for --midpoint-correction "
                    "(repeatable). Default: consider all colliders used for "
                    "collision; typically pass the same names as "
                    "--adaptive-substep-collider")
    ap.add_argument("--midpoint-correction-samples", type=int, default=1,
                    metavar="N",
                    help="number of sample points checked along each cloth "
                    "segment for --midpoint-correction (default 1 = the "
                    "midpoint only). Values above 1 check N interior points "
                    "(e.g. 2 checks the 1/3 and 2/3 points) and distribute "
                    "the push to the two real bones proportionally; cost "
                    "scales roughly linearly with N. The push direction is "
                    "always horizontal (perpendicular to gravity), never "
                    "up/down, regardless of this setting")
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
                collision_margin=a.collision_margin,
                force_no_collision_names=a.force_no_collision,
                allowed_collider_names=a.allowed_collider,
                hem_extra_margin=a.hem_extra_margin,
                adaptive_substep_threshold=a.adaptive_substep_threshold,
                adaptive_substep_max_n=a.adaptive_substep_max_n,
                adaptive_substep_collider_names=a.adaptive_substep_collider,
                midpoint_correction=a.midpoint_correction,
                midpoint_correction_iters=a.midpoint_correction_iters,
                midpoint_correction_margin=a.midpoint_correction_margin,
                midpoint_correction_collider_names=a.midpoint_correction_collider,
                midpoint_correction_samples=a.midpoint_correction_samples)
    except Exception as e:
        print("error:", e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
