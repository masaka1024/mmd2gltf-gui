<p align="center">
  <img src="icon.png" alt="mmd2gltf" width="160">
</p>

# mmd2gltf

![Release](https://img.shields.io/github/v/release/masaka1024/mmd2gltf-gui)
![Downloads](https://img.shields.io/github/downloads/masaka1024/mmd2gltf-gui/total)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![Python](https://img.shields.io/badge/Python-3-blue.svg)
![deps](https://img.shields.io/badge/deps-stdlib%20only-brightgreen.svg)

**[日本語](README.md) | English**

A tool that converts MMD PMX models (plus VMD motions) to **glTF 2.0 (.glb)** as faithfully as possible. Works as **both a GUI and a CLI**. Runs on the Python standard library alone; Pillow is only needed for texture conversion (BMP/TGA/sph/spa → PNG).

Includes a **physics-baking feature** that bakes rigid-body physics (hair, skirts, ties, etc.) into bone animation, so secondary motion plays back naturally even in glTF viewers that have no physics engine.

The real goal of this tool is not just viewer-oriented conversion, but **carrying MMD data out to other formats and environments as accurately as possible** (see [Goal of this tool](#goal-of-this-tool)).

> **▶ Quick start (Windows):** [Download the latest EXE](https://github.com/masaka1024/mmd2gltf-gui/releases/latest) — no Python required.

<p align="center">
  <img src="docs/screenshot.png" alt="mmd2gltf GUI" width="640">
</p>

## Screenshots

Conversion example: Tda-style Hatsune Miku Append / Miku V4X (models by Tda), imported into Unity via the [Unity importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer). Converted with the latest state of the Python version (as of 2026/07/19 18:00 JST).

<p align="center">
  <img src="docs/screenshot_tda_unity.png" alt="Tda-style Miku Append and V4X displayed in Unity" width="640">
</p>

> The model data itself is NOT included in this repository. Used under the [Piapro Character License](https://piapro.jp/license/pcl/summary) and each model's terms of use.
> © Crypton Future Media, INC. www.piapro.net

> ⚠️ **Physics (hair/skirt dynamics) reproduction is still a work in progress.** We keep tuning to bridge the engine gap between MMD (Bullet-style) and Unity (PhysX), but the motion is not yet on par with MMD itself. Please be gentle 🙏 (ideas and PRs are very welcome!)

## Verified models

Conversion and display have been verified with the following models (obtain the model data from each distribution page):

| Model | Author | Main verification points |
|---|---|---|
| Tda-style Hatsune Miku Append Ver1.10 | Tda | Material colors & sphere maps, semi-transparent overlays (forehead hair shadow / cheek), physics & lilToon reconstruction in Unity |
| Tda-style Hatsune Miku V4X Ver1.00 | Tda | Alpha classification of shared textures, render order of translucent materials (glasses / lenses) |
| IA (PMX model) | Omiya ([bowlroll](https://bowlroll.net/file/81272)) | Blend restoration of translucent hair (high-alpha), soft eyebrow / eye-lining translucency, physics baking (incl. skirt silhouette tuning and anti-penetration options) |

## Table of contents

- [Screenshots](#screenshots)
- [Verified models](#verified-models)
- [Goal of this tool](#goal-of-this-tool)
- [Features](#features)
- [Requirements](#requirements)
- [Windows EXE (no Python needed)](#windows-exe-no-python-needed)
- [Installing the Python version](#installing-the-python-version)
- [Usage](#usage)
- [IK handling](#ik-handling)
- [Physics baking](#physics-baking)
- [Option reference](#option-reference)
- [Viewer compatibility notes](#viewer-compatibility-notes)
- [What gets converted (fidelity)](#what-gets-converted-fidelity)
- [Limitations](#limitations)
- [Tests](#tests)
- [Project layout](#project-layout)
- [License](#license)

## Goal of this tool

The primary purpose of mmd2gltf is to **carry MMD data out to other formats and environments as accurately as possible**. glTF (.glb) is used as the "container" for that journey.

Think of the converted glTF as a **showcase holding an MMD asset**: through the glass — that is, in any glTF viewer — you can watch the model and its motion as-is, while the original data (`extras.mmd`) sits inside, untouched, as raw values.

To achieve this, every converted file has a **two-layer structure**:

1. **Preservation layer (`extras.mmd`)** — Everything glTF cannot express — rigid bodies, joints, IK settings, append/grant parents, toon / sphere-map / edge material settings, and more — is **fully preserved as raw PMX values**. A receiving application (a game engine or another tool) can reconstruct the original MMD model's structure from this layer with high fidelity.
2. **Baked layer (standard glTF animation)** — Motion and secondary physics (hair, skirts) are baked into standard keyframes so the model "just looks right" in ordinary glTF viewers that have no physics engine. This layer is an approximate fallback by design.

Physics baking never discards the preservation layer, so you can have both: viewers play the baked motion as-is, while engines rebuild real physics from the raw data.

There is no reverse converter (glTF → PMX) yet. But everything needed to restore the model is kept in `extras.mmd` as raw values. **If you ever want to touch the original again — try building that restoration tool.** The Unity importer is a working example of such a reconstruction, and the data layout is documented in `mmd2gltf/physicsGltf_schema.md`.

> **About usage terms:** The model's and motion's usage terms still apply after conversion. Because `extras.mmd` contains the original data in full, **distributing a converted file is equivalent to distributing the original data**. A model that prohibits redistribution cannot be distributed after conversion either.

### Proof of concept: the Unity importer

To verify that this design actually works, an editor extension that reads `extras.mmd` and reconstructs the model inside Unity was developed. The following has been confirmed on multiple models:

- **Rebuilding physics (secondary motion) from the rigid-body / joint data** — Rigidbody / ConfigurableJoint components are set up on the bones, so hair and skirts sway under Unity's real physics engine
- **Restoring MMD-style toon rendering** from the toon / sphere-map / shared-toon / outline material settings (based on lilToon)

In other words, a .glb produced by this tool is not only viewable as-is — it carries **enough information for an engine to bring back real physics and toon shading**.

> The importer is published as a separate repository: **[mmd2gltf-unity-physics-importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer)** (requires UniGLTF / lilToon, for URP projects; README in Japanese)

## Features

- **No extra libraries required** — runs on the standard library alone (Pillow is optional, for texture conversion only).
- **Full PMX 2.0/2.1 parsing** — meshes, skinning, materials, morphs, and physics data, all sections.
- **VMD motion baking** — evaluates Bézier interpolation and bakes every frame in MMD's exact deformation order (deform hierarchy → append/grant → CCD-IK with axis limits).
- **Physics baking** — simulates rigid-body physics for hair, ties, skirts, etc. and bakes the result into bone keyframes, so secondary motion works in viewers without a physics engine. Includes a range of options for skirt silhouette tuning and penetration mitigation.
- **Fine-grained IK control, 4 modes** — from full bake to "don't solve at all" (see [IK handling](#ik-handling)).
- **Nothing that glTF can't express is lost** — rigid bodies, joints, IK settings, append parents, and more are fully preserved as raw data under `extras.mmd`. Even when physics baking is on, the original physics data stays intact, so a game engine (e.g. Unity) can rebuild real physics from it (see the published [Unity importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer)).
- **GUI and CLI** — the GUI supports Japanese/English switching and drag & drop.
- **Prebuilt Windows EXE** — download and run without installing Python.

## Requirements

- **Windows EXE** — no Python installation needed; download and run.
- **Python version** — Python 3 (standard library only)
  - Optional: **Pillow** (needed when textures are not PNG/JPG)
  - Optional: **tkinterdnd2** (for drag & drop in the GUI)

## Windows EXE (no Python needed)

If you'd rather not install Python, use the prebuilt EXE.

1. Download the latest zip from the [Releases](https://github.com/masaka1024/mmd2gltf-gui/releases) page.
2. Extract it and double-click `mmd2gltf_gui.exe` (self-contained) to launch the GUI.
3. From there it's the same as the GUI described in [Usage](#usage). Pillow, tkinterdnd2, and numpy are bundled, so no extra installs are needed (drag & drop works in the EXE too).

> **About the first-launch warning**
> This is unsigned, individually developed software, so Windows SmartScreen may show
> "Windows protected your PC" on first launch. Click "More info" → "Run anyway".
> Some antivirus software may also flag PyInstaller-built EXEs as false positives;
> the full source is published in this repository.

## Installing the Python version

Just clone the repository and install the optional libraries as needed.

```bash
# Optional: when textures are not PNG/JPG
pip install Pillow

# Optional: for drag & drop in the GUI (in a uv environment: uv pip install tkinterdnd2)
pip install tkinterdnd2
```

## Usage

### GUI

```bash
python gui.py
```

You can pick files and set the main options (unlit, double-sided, morph storage mode, alpha mode) from the window. Open "Advanced settings" to configure IK options, step, animation name, and **physics baking**. Conversion runs in the background with a live log. The only dependency is the standard library's tkinter.

- You can **drag & drop** files from Explorer onto the PMX/VMD fields (requires `tkinterdnd2`; the "Browse..." buttons still work without it).
- The dropdown at the top right switches between **Japanese/English** at any time (auto-selected from your OS locale on first launch).

### CLI

```bash
python -m mmd2gltf model.pmx -o model.glb
python -m mmd2gltf model.pmx --vmd motion.vmd -o dance.glb

# Also bake hair/skirt physics
python -m mmd2gltf model.pmx --vmd motion.vmd --bake-physics --bake-target all -o dance.glb
```

### From Python

```python
from mmd2gltf import convert
```

## IK handling

When baking a VMD motion, you can choose one of 4 ways to handle IK (inverse kinematics for legs, arms, etc.). **The default full bake is usually what you want**; switch modes when the feet move in unintended ways.

| Mode | Flag | Behavior |
| --- | --- | --- |
| **Full bake (default)** | `--vmd FILE` only | Solves all IK and bakes it into the animation (30fps). IK on/off keys inside the VMD are respected. |
| **Don't solve any IK** | `--no-ik` | Doesn't solve IK at all during baking; outputs only the raw FK curves. |
| **Disable specific IK** | `--disable-ik NAME` | Disables only IK bones whose names contain NAME (repeatable). E.g. `--disable-ik 足` disables just the leg/toe IK. |
| **Ignore VMD IK keys** | `--ignore-vmd-ik` | Ignores IK on/off keys inside the VMD and solves anyway. Useful when a full-key motion sets leg IK to OFF, which the default would honor by not solving. |

**Quick guide**

- Ordinary dance motion → the default (`--vmd` only) is fine.
- Want to inspect the raw curves, or don't like the IK-solved result → `--no-ik`.
- Want to disable only the legs / only the arms → partial selection like `--disable-ik 足`.
- Feet don't track correctly because a full-key motion sets leg IK OFF in the VMD → `--ignore-vmd-ik`.

## Physics baking

With `--bake-physics`, MMD rigid-body physics (hair, ties, skirts, and other secondary motion) is solved with a lightweight physics simulation (a PBD spring model) and the result is **baked into bone keyframes**. Secondary motion then plays back naturally even in glTF viewers that have no physics engine.

> **Design philosophy:** this bake is an approximation aimed at "plausible, modern-looking sway for environments without a physics engine". It does not try to perfectly reproduce MMD's own Bullet physics. If you need faithful physics, use the raw rigid-body/joint data preserved under `extras.mmd` and rebuild it in an engine such as Unity.

### Basic usage

```bash
# Bake hair only (default)
python -m mmd2gltf model.pmx --vmd motion.vmd --bake-physics -o out.glb

# Bake everything including skirts
python -m mmd2gltf model.pmx --vmd motion.vmd --bake-physics --bake-target all -o out.glb
```

- `--bake-target hair` (default) — chain-like secondary parts only (hair, ties, etc.).
- `--bake-target all` — all dynamic rigid bodies including skirts. Skirts go through a cloth solver that preserves ring structures and includes collision handling against body colliders (legs, hips, etc.) to prevent penetration.

### Tuning the sway

| Option | Default | Effect |
| --- | --- | --- |
| `--hair-drag F` | 0.85 | Velocity retention (0–1). Higher = floppier, lower = stiffer. |
| `--hair-stiffness F` | 1.5 | Restoring force toward the rest shape. |
| `--hair-gravity F` | 0.02 | Gravity strength. 0 keeps the rest shape. |
| `--collision-bounce F` | 0.0 | Extra bounce when a hair/cloth particle is pushed out of a collider. 0 = unchanged behavior; higher = springier on contact. |
| `--collision-margin F` | 0.0 | Clearance kept between cloth and body colliders, in glTF units. The skirt now gets its clearance from the two measurement-based options below, so this defaults to 0. **Hair, neckties and other chains only get this one**, so raise it to around 0.003 if hair clips into the head. |
| `--hem-extra-margin F` | 0.0 | Extra clearance applied **only toward the hem** of each skirt chain. It interpolates smoothly from the root (waist, 0) to the hem (1), so it suppresses hem clipping without changing the waist silhouette. |

### Skirt silhouette tuning

The point-based solver (default) treats the skirt as chains of point masses, whereas in MMD itself the skirt is made of plate-shaped rigid bodies rotating around joints. That difference can show up as a "sliced tube" staircase look or a stiff, origami-like motion. The options below each target one of those symptoms. All of them default to OFF, so the output is unchanged unless you opt in.

| Option | Default | Symptom & effect |
| --- | --- | --- |
| `--segment-aware-collision` | OFF | Resolves collider push-out against each parent-to-child **segment** (line) instead of a single particle. Tackles the "staircase" look (one ring popping out when a leg lifts) at its root cause (skirt only; hair is unaffected). |
| `--skirt-bone-twist` | OFF | Keeps each skirt bone's real rotation (mesh twist) instead of always forcing it to identity. Fixes the "bulge at the midpoint of a segment, even though the bone positions are correct" artifact. The rotation used is already safety-clamped to a max of 60° change per frame (skirt only). |
| `--vertical-spread-scale F` | 0.5 | Distributes part of a collision push-out to the neighboring rings above and below in the same chain, so the bend spreads smoothly along the leg's angle instead of kinking at one ring. **Above 1.0 the neighbouring rings move further than the ring that was hit, so each contact is amplified as it spreads and the skirt flares like an umbrella. Stay within 0.5-1.0.** |
| `--rb-size-margin-scale F` | 1.0 | Derives extra clearance from each skirt rigid body's own PMX box size (smallest side = the panel's thickness). In MMD itself the rigid bodies' volume produces a natural, snug fit "for free", while bone-position collision treats particles as zero-size points; this reads the size back out of the model to approximate the same effect. In MMD the box collides on its **face**, so its centre always stops half a thickness away from the collider — which means 1.0 simply reuses the thickness the author modelled, and is a physically grounded default. |
| `--lateral-slack-scale F` | 6.0 | Allows slack in the skirt's lateral ring constraint (between adjacent panels at the same height), derived from the real PMX joint's translation limits (linearLimitMin/Max). MMD itself lets the distance move freely inside that range and only stops at the edges, whereas this solver used to pull it back to exactly the rest length every step — which can look stiff and origami-like on flare skirts. Start around 1.0 (the verified models use 6.0). |

**Recommended settings** — arrived at by measuring penetration and skirt opening on two models (IA and Ponpu-cho style Hatsune Miku).

```bash
python -m mmd2gltf model.pmx --vmd motion.vmd --bake-physics --bake-target all \
  --segment-aware-collision --skirt-bone-twist --symmetrize-colliders \
  --hem-extend-scale 1.0 --adaptive-substep-threshold 0.5 \
  -o out.glb
```

**The GUI launches with most of these values already filled in.** Only two need enabling under "Advanced settings": hem extension (1.0) and adaptive substep (threshold 0.5).

Measured figures for this configuration are listed under [Why penetration cannot be reduced to zero](#why-penetration-cannot-be-reduced-to-zero).

### Clearance measured from the mesh

A skirt is represented by a handful of plate-shaped rigid bodies. The bones sit at a position that stands for each plate, and the mesh at the valley of a fold lies further inside than that. The bake pushes the bones (particles) outside the colliders, so **the cloth can look like it goes through the legs even when nothing is penetrating numerically**. With only a few plates standing in for fabric, that difference is unavoidable.

`--drape-depth-scale` measures that sag directly from the PMX mesh and uses it as clearance.

| Option | Default | Effect |
| --- | --- | --- |
| `--drape-depth-scale F` | 1.0 | Use the mesh-measured drape depth as clearance. 1.0 uses the measured value as is, 0 disables it. The value is per position, so it varies automatically from waist to hem. |
| `--no-drape-probe` | — | Disable moving the midpoint-correction samples inward to the drape depth (for comparison). |

How large that difference is varies a lot between models, and within a single skirt. On one of the verified models it was 0.089 at the waist against 0.330 at the hem (PMX units) — a **3.7x gradient** that no single uniform clearance can match without being too much at the waist or too little at the hem. A measured value carries the gradient with it.

Rigid bodies whose drape depth could not be measured (too few vertices weighted to that bone, etc.) keep using the `--rb-size-margin-scale` value instead.

### Correcting left/right asymmetry

Collider positions and sizes sometimes carry a small left/right difference. In MMD itself (Bullet physics) the rigid bodies collide as volumes, so a difference of that size never surfaces. This bake, on the other hand, treats bones as points and adds clearance around them to keep the shape. **Once the difference is comparable to that clearance, the skirt can look pushed out on one side only** — it surfaces because of the approximation, not because of the model.

On one of the verified models the gap was 0.046 at the thigh and 0.067 at the lower leg, relative to their bones (the bones themselves were perfectly symmetric).

| Option | Default | Effect |
| --- | --- | --- |
| `--symmetrize-colliders` | OFF | Mirror-averages the position, size and rotation of each left/right collider pair relative to its bone, correcting **only the bake input**. **The data written to `extras.mmd` is left untouched.** Some models shape the two sides differently on purpose, so this is off by default. |

### Physics report (for investigation)

| Option | Default | Effect |
| --- | --- | --- |
| `--physics-report` | OFF | After baking, logs for each skirt ring: how far it opened compared with its rest radius, which collider it was closest to, how often the collision push fired, and the estimated mesh penetration rate and depth. |

Useful when tracking down odd cloth behaviour. The opening timeline in particular (the average over ten equal slices of the motion) tells you at a glance whether the skirt opens on contact and recovers, or opened once and never came back. It makes baking slightly slower.

### Push-out behaviour

| Option | Default | Effect |
| --- | --- | --- |
| `--collision-velocity-damp F` | 1.0 | How much of the velocity created by a push-out is cancelled. In Verlet integration moving a position also changes the velocity, so 1.0 gives a fully inelastic response that fixes the position without touching the velocity. 0 restores the old behaviour. |
| `--margin-rest-clamp` | OFF | Cap the clearance of each particle-collider pair at the distance actually available in the rest pose. Off by default because it trims the measurement-based clearance, but kept as an escape hatch for models where the cloth sits extremely tight to the body. |

### Anti-penetration options

With fast motion, two kinds of see-through can occur: (1) a frame's movement is large enough to jump straight past the collision check (tunneling), and (2) the bones (joint points) themselves never penetrate, but the mesh surface between two adjacent bones visually crosses through the body.

| Option | Default | Effect |
| --- | --- | --- |
| `--hem-extend-scale F` | 0.0 | Virtually extends each skirt chain's last (hem) segment by F × half the hem rigid body's height for collision testing. The hem bone sits at the center of a plate-shaped rigid body, so the actual cloth reaches further down than the bone; this compensates for penetration in that unreached part (1.0 = the rigid body's real size). |
| `--adaptive-substep-threshold F` | disabled | Targets (1). Automatically subdivides only those frames where the anchor+collider relative movement exceeds F × the nearest collider's radius. Typical values: 0.5–0.75. Leaving it unset keeps the behavior fully unchanged. |
| `--adaptive-substep-max-n N` | 4 | Cap on how many substeps a single frame can be split into. |
| `--adaptive-substep-collider NAME` | all colliders | Restricts which colliders trigger substepping (repeatable). Passing only leg/knee colliders makes fast arm motion not trigger it. |
| `--midpoint-correction` | OFF | Targets (2). An extra pass that nudges the two real bones apart when the midpoint of the mesh segment between adjacent skirt bones is found inside a collider — a case bone-level collision fundamentally cannot detect. Measured to cut penetration depth/area by roughly 60–75%, but does not eliminate it entirely. |
| `--midpoint-correction-iters N` | 2 | Relaxation iterations (cost overhead is small even at 6). |
| `--midpoint-correction-margin F` | 0.0 | Clearance used by this correction. |
| `--midpoint-correction-collider NAME` | all colliders | Restricts which colliders are considered (repeatable). Typically pass the same names as `--adaptive-substep-collider`. |
| `--midpoint-correction-samples N` | 1 | Number of sample points checked along each segment. 1 = the midpoint only; higher values check N interior points (e.g. 2 checks the 1/3 and 2/3 points) and distribute the push to the two bones proportionally. The push direction is always horizontal (perpendicular to gravity). |

### Why penetration cannot be reduced to zero

The bake writes its result as **bone keyframes**, and the mesh follows through skinning. What this bake can control is where the bones are, not where each individual vertex ends up.

A skirt is represented by a handful of plate-shaped rigid bodies. The bones stand for those plates, and the mesh at the valley of a fold lies further inside. Keeping the bones outside the colliders still leaves that inner cloth free to cross the legs. `--drape-depth-scale` measures the difference from the mesh and feeds it back as clearance, but the difference also changes with the pose, so it can be narrowed and not closed.

MMD itself (Bullet physics) represents the skirt with plate-shaped rigid bodies in the same way, and neither side takes per-vertex separation as a premise. How a skirt reads on screen is shaped by how its rigid bodies and joints are tuned.

For reference, here are measured figures with the recommended settings (share of sampled frames where the fold of the cloth ended up inside a collider, and how deep, in glTF units):

| Model | Waist ring | Middle | Hem | Mean depth at the hem |
| --- | --- | --- | --- | --- |
| IA | 0.1% | 0.0% | 11.3% | 0.0045 |
| Ponpu-cho style Miku | 3.8% | 0.6% | 3.3% (lowest ring 0.6%) | 0.0018 |

The upper rings can be cleared almost entirely; what remains is the hem. A depth of 0.0045 works out to roughly 4 mm on a 1.6-unit-tall figure.

**If you need true non-penetration**, use the rigid body and joint data preserved in `extras.mmd` and build a vertex-level cloth simulation on the receiving side — Unity's Cloth, Unreal Engine's Chaos Cloth, and so on. The bake layer is an approximation for environments that have no physics engine at all.

### Experimental: cloth algorithm switch

`--cloth-algorithm` switches the skirt solver:

- `points` (default) — the existing point-mass chain. This is the main line and supports all the options above.
- `rigid_chain` (experimental) — simulates each ring's orientation directly (inertia + gravity torque + stiffness + the real PMX joint's per-axis angle limits). No collision, substep, or midpoint-correction support yet.
- `rigid_body` (experimental) — a rigid-body chain with both translation and angular velocity. Tuned via the `--rigid-body-*` parameters.

`points` is the right choice for normal use. See `--help` for the experimental algorithms' detailed parameters.

### Collision modes (fixing clipping and "umbrella" skirts)

On some models, internal rigid-body setups or motions more extreme than the model was designed for can push the skirt outward into an umbrella shape, or pin it into a body part. Two escape hatches are provided:

| Mode | Flag | Behavior |
| --- | --- | --- |
| **Normal (denylist)** | `--force-no-collision NAME` (repeatable) | Makes only the named rigid bodies fully non-colliding; everything else follows the PMX non-collision group settings. Use when you know a specific collider is causing trouble. |
| **Restricted (allowlist)** | `--allowed-collider NAME` (repeatable) | Treats **only** the named rigid bodies as colliders and ignores everything else, bypassing the PMX group settings entirely. Same idea as VRM SpringBone / VRChat PhysBones per-chain collider scoping. |

The GUI makes this easier under "Advanced settings":

- **Collision mode** — radio buttons switch between "Normal / Restricted", with a comma-separated name field.
- **Show rigid body names...** — parses the selected PMX and lists rigid bodies in two groups (static/colliders vs. dynamic/sway parts); double-click to add a name to the field.
- **Use VRM-compatible mode** — a one-click preset that auto-detects leg-area colliders and switches to Restricted mode.

The PMX group settings (group/noCollisionMask) themselves are also read and honored during baking; the modes above are overrides for when those settings aren't enough.

> **About that 16-bit field:** PMX calls it the "non-collision group flag", but **a set bit means the body DOES collide with that group** — it is Bullet's collision filter mask verbatim. The checkboxes in PMXEditor are shown inverted (checked = no collision), so reading the field by its name gives you exactly the wrong answer. three.js's mmd-parser calls it `groupTarget` and saba calls it `m_collisionGroup`; both avoid the word "non-collision". This tool now follows the same interpretation (the value written to `extras.mmd` is still the raw PMX one — see `mmd2gltf/physicsGltf_schema.md`).

### Inspecting rigid bodies visually

`tools/mmd_physics_inspector.html` is a standalone tool for looking inside a converted GLB in your browser — just drag and drop the file onto it, no installation required.

It renders the rigid bodies, joints and collision groups stored in `extras.mmd` in 3D. Clicking a rigid body shows its shape, size, mode, bound bone, the groups it collides with, and how many other bodies it can collide with. When you are unsure how a model is put together, it answers "where is this collider" and "is it actually set to collide" in a couple of minutes.

## Option reference

### General

| Option | Description |
| --- | --- |
| `--vmd FILE` | Bake a VMD motion as a glTF animation (with IK solving, 30fps) |
| `--no-ik` | Don't solve any IK during baking (raw FK curves only) |
| `--disable-ik NAME` | Disable only IK bones whose names contain NAME (repeatable; e.g. `--disable-ik 足` for leg/toe IK) |
| `--ignore-vmd-ik` | Ignore IK on/off keys inside the VMD (respected by default; e.g. full-key motions that set leg IK OFF would otherwise not be solved) |
| `--step N` | Sample every N frames to reduce file size (default 1 = every frame) |
| `--unlit` | Add `KHR_materials_unlit` to materials (closer to MMD's toon look) |
| `--no-extras` | Don't write `extras.mmd` (see below) |
| `--anim-name NAME` | Name for the glTF animation |
| `--morph-mode MODE` | Morph storage: `sparse` = compact (default), `dense` = maximum compatibility (use if faces break in viewers with poor sparse support), `none` = no morphs |
| `--alpha-mode MODE` | `auto` (default) analyzes each texture's alpha distribution and picks OPAQUE/MASK/BLEND, avoiding false BLEND from unused transparent regions of skin textures (fixes see-through / inside-out faces). `opaque`/`mask`/`blend` force one mode for all materials |
| `--force-double-sided` | Render all materials double-sided (same behavior as three.js MMDLoader; use if hair/skirt backfaces disappear) |
| `--scale F` | Uniform scale from MMD units to glTF units (meters; default 0.08). MMD models are conventionally authored at ~1 unit ≈ 8cm (a 160cm character is about 20 units), so leaving it at `1.0` makes models appear ~12.5x too large in glTF viewers. Applied to vertex/bone positions, SDEF params, morph position deltas, and baked animation translations; normals, UVs, rotations, and the raw data under `extras.mmd` (rigid bodies, joints, etc.) are unaffected (the factor used is recorded in `extras.mmd.unitScale`). Adjust (e.g. `--scale 1.0`) if the source model uses a different unit convention |
| `--no-custom-attrs` | Don't emit MMD-specific vertex attributes (`_SDEF_C`/`_SDEF_R0`/`_SDEF_R1`/`_ADDUV1..4`/`_EDGESCALE`/`_WEIGHTTYPE`). **Use this if Blender's built-in glTF importer errors on these attribute names** (the data still remains under `extras.mmd`, so nothing is lost) |

### Physics baking

See the sections under [Physics baking](#physics-baking) for symptom-based explanations of each option.

| Option | Description |
| --- | --- |
| `--bake-physics` | Bake rigid-body physics into bone keyframes (PBD spring sim); fixes frozen/IK-locked hair, ties, etc. |
| `--bake-target MODE` | Which rigid bodies to bake: `hair` = hair only (default), `all` = all dynamic bodies incl. skirts (cloth solver handles ring structures) |
| `--hair-drag F` | Velocity retention 0–1 (default 0.85; higher = floppier) |
| `--hair-stiffness F` | Rest-shape restoring force (default 1.5) |
| `--hair-gravity F` | Gravity strength (default 0.02; 0 keeps the rest shape) |
| `--collision-bounce F` | Extra bounce when pushed out of a collider (default 0.0 = unchanged; higher = springier on contact) |
| `--collision-margin F` | Clearance kept between cloth and body colliders, in glTF units (default 0.0; the skirt gets its clearance from the two measurement-based options, and this is the only one that reaches hair and neckties). Increase if the skirt visually touches/clips the legs; 0 = push exactly to the collider surface |
| `--hem-extra-margin F` | Extra clearance added on top of `--collision-margin`, interpolated smoothly from root (0) to hem (1) of each skirt chain, so the waist side is unaffected (default 0.0 = off) |
| `--segment-aware-collision` | Resolve collision push-out per parent-to-child segment instead of per particle (skirt only). Root-cause fix for the "staircase" look (default OFF) |
| `--skirt-bone-twist` | Keep each skirt bone's real rotation (mesh twist) instead of forcing identity (default OFF). Uses values already safety-clamped to 60°/frame |
| `--vertical-spread-scale F` | Distribute push-out to neighboring rings above/below for smoother bends (default 0.5; above 1.0 the spread is amplified and the skirt flares) |
| `--rb-size-margin-scale F` | Extra clearance derived from each skirt rigid body's own box size (default 1.0 = the thickness as modelled; 0 = off) |
| `--lateral-slack-scale F` | Allow slack in the lateral ring constraint, derived from the real PMX joint limits (default 6.0; 0 = hard constraint) |
| `--hem-extend-scale F` | Virtually extend the last (hem) segment by F × half the hem rigid body's height for collision (default 0.0 = off; 1.0 = the body's real size). Compensates for cloth reaching below the hem bone |
| `--adaptive-substep-threshold F` | Auto-subdivide only fast-motion frames to prevent tunneling (default = disabled; typical 0.5–0.75) |
| `--adaptive-substep-max-n N` | Cap on substeps per frame (default 4) |
| `--adaptive-substep-collider NAME` | Restrict which colliders trigger substepping (repeatable; default = all) |
| `--midpoint-correction` | Extra pass nudging bones apart when a mesh segment's midpoint is inside a collider (default OFF; cuts penetration by ~60–75%) |
| `--midpoint-correction-iters N` | Relaxation iterations for the correction (default 2) |
| `--midpoint-correction-margin F` | Clearance used by the correction (default 0.0) |
| `--midpoint-correction-collider NAME` | Restrict which colliders are considered (repeatable; default = all) |
| `--midpoint-correction-samples N` | Sample points per segment (default 1 = midpoint only; higher checks N interior points, distributing the push proportionally) |
| `--force-no-collision NAME` | Force the named rigid body to be fully non-colliding, overriding its PMX group/noCollisionMask (denylist escape hatch; repeatable). Use when a specific collider pins/stretches cloth in extreme poses despite the PMX data looking correct |
| `--allowed-collider NAME` | Switch collision to allowlist mode: only the named rigid bodies are treated as colliders at all (repeatable; ignores PMX group/noCollisionMask entirely once any is given). Mirrors VRM SpringBone / VRChat PhysBones-style per-chain collider scoping |
| `--drape-depth-scale F` | Use the mesh-measured drape depth as the skirt's clearance (default 1.0 = as measured, 0 = off). The value is per position, so it varies from waist to hem automatically |
| `--no-drape-probe` | Disable moving the midpoint-correction samples inward to the drape depth (for comparison) |
| `--symmetrize-colliders` | Mirror-average left/right collider pairs so only the bake input is symmetric (default OFF). The data in `extras.mmd` is unchanged |
| `--collision-velocity-damp F` | How much of the velocity created by a push-out is cancelled (default 1.0 = fully inelastic, 0 = old behaviour) |
| `--margin-rest-clamp` | Cap clearance at the distance available in the rest pose (default OFF) |
| `--physics-report` | After baking, log per-ring skirt opening, nearest collider, push frequency and estimated penetration (default OFF) |
| `--cloth-algorithm MODE` | Skirt solver: `points` (default, main line) / `rigid_chain` (experimental) / `rigid_body` (experimental). See `--help` for the experimental algorithms' `--rigid-chain-*` / `--rigid-body-*` parameters |

## Viewer compatibility notes

- macOS Quick Look / Preview (RealityKit) doesn't support glTF morph targets at all. Morphs "missing" there is a viewer limitation; the file itself is fine.
- Viewers/loaders with incomplete sparse-accessor support can break meshes when morphs are applied (see-through faces, mouth morphs appearing to deform the wrong area, etc.). In that case convert with `--morph-mode dense` (larger file). **`dense` is also recommended when importing into Unity via UniGLTF/UniVRM.** Even in sparse mode, a zero-filled base bufferView is included, so loaders without sparse support degrade safely to "morphs disabled".
- Verified viewers: three.js-based (gltf-viewer.donmccurdy.com), Babylon.js Sandbox (sandbox.babylonjs.com), and Blender 3.x+ glTF importer.

## What gets converted (fidelity)

Directly representable in glTF:

- Meshes (vertices, normals, UVs, per-material primitive splits)
- Skinning (BDEF1/2/4; SDEF/QDEF approximated with linear blending, original parameters preserved)
- Bone hierarchy (PMX bone order = skin.joints order, so indices stay compatible)
- Vertex morphs → morph targets (compact via sparse accessors), UV morphs → `TEXCOORD_0` targets, group morphs → expanded into composed targets
- Materials (diffuse → baseColor, double-sided flag, alpha-based BLEND detection, embedded textures)
- VMD motion: Bézier interpolation evaluated and baked per frame in MMD's exact deformation order (deform hierarchy → append/grant → CCD-IK with axis limits). Morph keys become a weights animation
- Physics (optional, `--bake-physics`): rigid-body physics simulated and baked as bone rotation + translation keyframes. PMX non-collision groups (group/noCollisionMask) are honored

Everything glTF has no concept for is preserved in full under `extras.mmd` (raw PMX values, MMD left-handed coordinates):

- Rigid bodies and joints (physics), IK settings, append parents, fixed/local axes, display frames
- Bone morphs, material morphs, flip/impulse morph contents
- Material sphere-map / toon / edge / ambient / specular settings and memos
- Per-vertex data kept as custom attributes: `_ADDUV1..4`, `_EDGESCALE`, `_SDEF_C/_SDEF_R0/_SDEF_R1`, `_WEIGHTTYPE`

Coordinate conversion: positions/normals `(x,y,z)→(x,y,-z)`, quaternions `(x,y,z,w)→(-x,-y,z,w)`, triangle winding reversed.

## Limitations

- PMD (legacy format) and PMX 2.1 soft bodies are not supported (convert to PMX with PMXEditor etc.)
- Physics baking is an approximation via a lightweight simulation and does not exactly reproduce MMD's Bullet physics. Collision is handled per bone (rigid body), not per mesh vertex, so with very fast motion the skirt's mesh surface can still visually pass through the body (tunneling can be mitigated with `--adaptive-substep-threshold` and mesh crossing with `--midpoint-correction`, though neither eliminates it entirely). Motions far more extreme than the model's rigid-body layout was designed for can also break down (this can happen in MMD itself as well). For faithful physics, rebuild it engine-side from the raw data in `extras.mmd`
- MMD's toon shading / sphere maps / edge rendering cannot be reproduced in glTF's PBR, so appearance is viewer-dependent (`--unlit` gets closer)
- Shared toon textures (toon01–10.bmp) ship with MMD itself and are not embedded (the index is preserved in extras)
- VMD camera, lighting, and self-shadow keys are out of scope

## Tests

```bash
python tests/make_test_data.py                # generate synthetic PMX/VMD
python -m mmd2gltf tests/test.pmx --vmd tests/test.vmd -o tests/test.glb
python tests/check_glb.py tests/test.glb      # structural validation
```

## Project layout

```
mmd2gltf/
  pmx.py        PMX 2.0/2.1 parser (all sections)
  vmd.py        VMD parser + Bézier interpolation
  animation.py  MMD-style deformation pipeline (append/grant, CCD-IK) and baking
  bake_hair.py  Physics baking (PBD spring/cloth solver, segment-aware collision, anti-penetration passes, collision modes)
  gltf.py       GLB builder (sparse accessor support)
  convert.py    Conversion core
  cli.py        CLI
  drape.py      Measures the drape depth of the cloth from the PMX mesh
tools/
  mmd_physics_inspector.html  Visual check for rigid bodies, joints and collision groups
  build_release.ps1           Builds the Windows EXE and packs the release zip
  release_pack.py             Verifies the payload and builds the zip (called by the above)
```

## License

The source code of this tool (mmd2gltf) is published under the MIT License. See [LICENSE](LICENSE) for details.

The Windows EXE bundles open-source components such as Python, Pillow, NumPy, tkinterdnd2, tkDnD, and Tcl/Tk. Their licenses are compiled in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
