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
                    help="velocity damping 0..1 (default 0.85). Each frame, "
                    "this fraction of the previous momentum is discarded "
                    "(inertia scales by 1-drag): lower = keeps more momentum "
                    "(stronger centrifugal swing on turns), higher = settles "
                    "faster")
    ap.add_argument("--hair-stiffness", type=float, default=1.5, metavar="F",
                    help="rest-shape restoring force (default 1.5)")
    ap.add_argument("--hair-gravity", type=float, default=0.02, metavar="F",
                    help="gravity strength (default 0.02; 0 keeps rest shape)")
    ap.add_argument("--collision-bounce", type=float, default=0.0, metavar="F",
                    help="extra bounce when a hair/cloth particle is pushed "
                    "out of a collider (default 0.0 = no bounce, unchanged "
                    "behavior; higher = springier on contact)")
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
    ap.add_argument("--hem-extend-scale", type=float, default=0.0, metavar="F",
                    help="virtually extend each skirt chain's last (hem) "
                    "segment downward for collision testing, by the tail rigid "
                    "body's half height (PMX box size[1]) times this factor "
                    "(default 0.0 = off; 1.0 = reach the bottom of the authored "
                    "box). Motivated by inspecting real models: the tail bone "
                    "sits at the center of the lowest rigid body plate, so the "
                    "authored collision volume extends half a plate below the "
                    "last particle. Effective only together with "
                    "--segment-aware-collision")
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
    ap.add_argument("--rb-size-margin-scale", type=float, default=0.0,
                    metavar="F",
                    help="derive extra clearance from each skirt rigid "
                    "body's own PMX box size (the smallest of size_x/y/z, "
                    "taken as the panel's thickness), scaled by this factor "
                    "and added on top of --collision-margin/--hem-extra-margin "
                    "(default 0.0 = off, unchanged behavior). Bone-position "
                    "collision treats particles as zero-size points, while "
                    "the PMX skirt rigid bodies are actually thin boxes, so "
                    "real MMD viewers get a snug, non-bulging fit 'for free' "
                    "from that volume; this approximates the same effect by "
                    "reading the box size back out of the model. Only "
                    "affects box-shaped (shape=1) rigid bodies within the "
                    "skirt/lower-body chain set; capsule/sphere jiggle bodies "
                    "are unaffected. Start around 1.0 and adjust to taste, "
                    "since whether PMX size values are half- or full-extent "
                    "can vary by tool/model")
    ap.add_argument("--lateral-slack-scale", type=float, default=0.0,
                    metavar="F",
                    help="allow slack in the skirt's lateral ring distance "
                    "constraint (between adjacent panels at the same height), "
                    "derived from the real PMX joint's linearLimitMin/Max "
                    "(default 0.0 = off, unchanged hard equality constraint). "
                    "Real MMD/Bullet solves these joints with spring "
                    "stiffness 0 and a translation limit -- free movement "
                    "inside that range, hard stop at the edges -- while this "
                    "solver always pulled the distance back to exactly "
                    "rest_len, which can look stiff/origami-like on flare "
                    "skirts. At 1.0, distances within [rest_len-slack, "
                    "rest_len+slack] are left uncorrected; only distances "
                    "outside that range are pushed back to the nearest edge "
                    "(not all the way to rest_len). Start around 1.0 and "
                    "adjust to taste")
    ap.add_argument("--vertical-spread-scale", type=float, default=0.0,
                    metavar="F",
                    help="distribute a collision-pushed particle's "
                    "displacement to its immediate vertical neighbors "
                    "(one ring up, one ring down) in the same chain "
                    "(default 0.0 = off, unchanged). Since this solver moves "
                    "one point at a time, a single ring pushed out by a leg "
                    "can look like a sliced tube, especially on bent-leg "
                    "poses, whereas real MMD's panels rotate around a joint "
                    "with very free rotation limits and follow the leg's "
                    "angle smoothly. This approximates that by leaking a "
                    "fraction of the push to neighboring rings so the bend "
                    "spreads out instead of kinking at one ring. Start "
                    "around 1.0 and adjust to taste")
    ap.add_argument("--cloth-algorithm", choices=["points", "rigid_chain", "rigid_body"],
                    default="points",
                    help="skirt solver: 'points' (default) is the existing "
                    "point-mass chain. 'rigid_chain' is an experimental "
                    "alternative that simulates each ring's orientation "
                    "directly (inertia + gravity torque + stiffness + the "
                    "real PMX joint's per-axis X/Y/Z angle limits), matching "
                    "how real MMD/Bullet panels rotate around a joint instead "
                    "of moving as bare points. Currently vertical drive + "
                    "lateral ring only: no collision, adaptive substep, or "
                    "midpoint correction support yet")
    ap.add_argument("--rigid-chain-gravity-power", type=float, default=20.0,
                    metavar="F",
                    help="gravity strength for --cloth-algorithm rigid_chain, "
                    "in radians of rotation per second (not the same units "
                    "as --gravity, which is a linear displacement). Default "
                    "20.0 was picked from real-data testing where the hem's "
                    "deviation angle saturates against the joint's own X-axis "
                    "limit around this value")
    ap.add_argument("--rigid-chain-stiffness-force", type=float, default=1.0,
                    metavar="F",
                    help="stiffness for --cloth-algorithm rigid_chain: "
                    "fraction of the orientation deviation relaxed back "
                    "toward rest per second (clamped 0..1 after *dt). "
                    "Default 1.0")
    ap.add_argument("--rigid-body-gravity", type=float, default=9.8,
                    metavar="F",
                    help="gravity acceleration magnitude for --cloth-algorithm "
                    "rigid_body (a real physical value, unlike --gravity's "
                    "per-frame nudge). Default 9.8 (Earth gravity)")
    ap.add_argument("--rigid-body-linear-damping", type=float, default=1.0,
                    metavar="F",
                    help="scale factor applied to each skirt rigid body's "
                    "OWN linear damping value from the PMX file (matching "
                    "Bullet's applyDamping). Default 1.0 uses the model's "
                    "real value as-is (e.g. 0.9 for this IA model); previous "
                    "versions used a flat 0.1 for every body regardless of "
                    "the model, which was found to under-damp the "
                    "simulation significantly")
    ap.add_argument("--rigid-body-angular-damping", type=float, default=1.0,
                    metavar="F",
                    help="scale factor applied to each skirt rigid body's "
                    "OWN angular damping value from the PMX file. Default "
                    "1.0 uses the model's real value as-is")
    ap.add_argument("--rigid-body-warmup-seconds", type=float, default=20.0/30.0,
                    metavar="F",
                    help="settling time (seconds, holding frame 0's pose) "
                    "for --cloth-algorithm rigid_body before real playback "
                    "starts, letting the skirt fall from bind pose to a "
                    "natural hang under gravity. Default matches the "
                    "existing 20-frame warmup (~0.67s). Increase this if "
                    "the initial drop from bind pose looks too abrupt")
    ap.add_argument("--rigid-body-iterations", type=int, default=4,
                    metavar="N",
                    help="Gauss-Seidel constraint solver iterations per "
                    "frame for --cloth-algorithm rigid_body. Default 4. "
                    "Using the PMX model's own (often high, e.g. 0.9) "
                    "damping values can require more iterations to converge "
                    "without the skirt ring collapsing; prefer raising this "
                    "over lowering damping away from the model's real "
                    "values, which would defeat the purpose of a physically "
                    "faithful simulation")
    ap.add_argument("--rigid-body-prime-full-motion", action="store_true",
                    help="for --cloth-algorithm rigid_body: play through the "
                    "whole motion once first (output discarded) before "
                    "recording the real pass from frame 0, using the settled "
                    "physics state as the new starting point. Mirrors real "
                    "MMD viewers, where the first playthrough's frame 0 "
                    "looks chaotic but the second playthrough (bones reset "
                    "to their frame-0 keyframes, physics keeps its settled "
                    "state) looks correct. Roughly doubles conversion time")
    ap.add_argument("--rigid-body-self-collision-scale", type=float, default=1.0,
                    metavar="F",
                    help="fraction of each pair's combined half-width used "
                    "as the minimum separation distance for skirt self-"
                    "collision. Default 1.0 (the full half-width sum). A "
                    "lower value was tried as a fix for the skirt getting "
                    "stuck near its initial pose, but the real cause turned "
                    "out to be a damping formula bug (see --rigid-body-"
                    "linear-damping); with that fixed, 1.0 gives clearly "
                    "better results")
    ap.add_argument("--rigid-body-max-linear-speed", type=float, default=20.0,
                    metavar="F",
                    help="clamp on each skirt rigid body's linear speed "
                    "(units/sec). Default 20.0. Mirrors Bullet's own "
                    "MAX_ANGVEL safety clamp in btRigidBody::"
                    "integrateVelocities, which this implementation lacked; "
                    "without it, a sudden large anchor jump (e.g. motion "
                    "start, or the transition from a rigid-body-prime-full-"
                    "motion priming pass to the recorded pass) can inject "
                    "runaway velocity that propagates and never settles")
    ap.add_argument("--rigid-body-max-angular-speed", type=float, default=30.0,
                    metavar="F",
                    help="clamp on each skirt rigid body's angular speed "
                    "(radians/sec). Default 30.0. See --rigid-body-max-"
                    "linear-speed")
    ap.add_argument("--rigid-body-substeps", type=int, default=2,
                    metavar="N",
                    help="split each 1/fps-second frame into N substeps for "
                    "--cloth-algorithm rigid_body, interpolating the anchor "
                    "bone's position/rotation between frames for the "
                    "in-between substeps. Default 2. Added after learning "
                    "the real MMD viewer runs at 60fps (double this tool's "
                    "default 30fps): a smaller timestep gives the same "
                    "iteration count less error to resolve each step, which "
                    "may help more than simply raising --rigid-body-"
                    "iterations. 1 disables substepping (one step per frame, "
                    "the original behavior)")
    ap.add_argument("--segment-aware-collision", action="store_true",
                    help="for --cloth-algorithm points (skirt only, does not "
                    "affect hair): resolve collider push-out against each "
                    "parent-to-child SEGMENT instead of a single particle. "
                    "When the collision point falls partway along a segment, "
                    "the push-out is distributed between the parent and "
                    "child particles instead of only moving whichever one "
                    "happened to be the one flagged as colliding. Aims at "
                    "the 'staircase' look (segments popping out unevenly, "
                    "e.g. when one leg lifts) at its root cause rather than "
                    "smoothing the result afterward like --vertical-spread-"
                    "scale does. Default off (identical to the existing "
                    "per-particle resolve_collisions)")
    ap.add_argument("--skirt-bone-twist", action="store_true",
                    help="for --cloth-algorithm points (skirt only, does not "
                    "affect hair): keep each skirt bone's real rotation "
                    "(mesh twist) instead of always forcing it to identity. "
                    "World position is unaffected either way (already baked "
                    "exactly via translation); this only controls how the "
                    "segment's tilt is conveyed to skin weighting. Off by "
                    "default because early testing found the raw rotation "
                    "could jump ~109 degrees in one frame at a q_from_to "
                    "singularity for tangled chain topologies (e.g. ring-"
                    "crossing joints) -- but that jump is already clamped by "
                    "simulate_step_cloth's own q_slerp_toward safety net "
                    "(max 60 degrees change per frame from the previous "
                    "frame), so enabling this should be safe while fixing "
                    "the 'bulge at the midpoint of a segment, not the bone "
                    "itself' artifact from discarding twist information "
                    "entirely")
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
                hem_extend_scale=a.hem_extend_scale,
                adaptive_substep_threshold=a.adaptive_substep_threshold,
                adaptive_substep_max_n=a.adaptive_substep_max_n,
                adaptive_substep_collider_names=a.adaptive_substep_collider,
                midpoint_correction=a.midpoint_correction,
                midpoint_correction_iters=a.midpoint_correction_iters,
                midpoint_correction_margin=a.midpoint_correction_margin,
                midpoint_correction_collider_names=a.midpoint_correction_collider,
                midpoint_correction_samples=a.midpoint_correction_samples,
                collision_bounce=a.collision_bounce,
                rb_size_margin_scale=a.rb_size_margin_scale,
                lateral_slack_scale=a.lateral_slack_scale,
                vertical_spread_scale=a.vertical_spread_scale,
                cloth_algorithm=a.cloth_algorithm,
                rigid_chain_gravity_power=a.rigid_chain_gravity_power,
                rigid_chain_stiffness_force=a.rigid_chain_stiffness_force,
                rigid_body_gravity=a.rigid_body_gravity,
                rigid_body_linear_damping=a.rigid_body_linear_damping,
                rigid_body_angular_damping=a.rigid_body_angular_damping,
                rigid_body_warmup_seconds=a.rigid_body_warmup_seconds,
                rigid_body_iterations=a.rigid_body_iterations,
                rigid_body_prime_full_motion=a.rigid_body_prime_full_motion,
                rigid_body_self_collision_scale=a.rigid_body_self_collision_scale,
                rigid_body_max_linear_speed=a.rigid_body_max_linear_speed,
                rigid_body_max_angular_speed=a.rigid_body_max_angular_speed,
                rigid_body_substeps=a.rigid_body_substeps,
                segment_aware_collision=a.segment_aware_collision,
                skirt_bone_twist=a.skirt_bone_twist)
    except Exception as e:
        print("error:", e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
