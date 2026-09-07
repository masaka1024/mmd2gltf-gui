<p align="center">
  <img src="icon.png" alt="mmd2gltf" width="160">
</p>

# mmd2gltf

![Release](https://img.shields.io/github/v/release/masaka1024/mmd2gltf-gui)
![Downloads](https://img.shields.io/github/downloads/masaka1024/mmd2gltf-gui/total)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![Python](https://img.shields.io/badge/Python-3-blue.svg)

**[日本語](README.md) | English**

<!-- mmd2gltf-ecosystem:start -->
> **The mmd2gltf ecosystem** — Convert an MMD model (PMX/VMD) to `.glb` once, and bring it into **Unity, Unreal Engine 5, or Blender** as is, with physics (secondary motion) and materials (toon/sphere). MMD-specific data that glTF cannot express is kept under `extras.mmd` as the original values, and each importer reads it to rebuild the model on the engine side.
>
> ```
> PMX / VMD
>    │  convert (mmd2gltf-gui or mmd2gltf-cs)
>    ▼
> .glb + extras.mmd   ← converted only once
>    │
>    ├─▶ Unity   … mmd2gltf-unity-physics-importer
>    ├─▶ UE5     … mmd2gltf-ue5-physics-importer
>    └─▶ Blender … mmd2gltf-blender-physics-importer
> ```
>
> | Goal | Use |
> |---|---|
> | Convert first (Windows EXE available) | [mmd2gltf-gui](https://github.com/masaka1024/mmd2gltf-gui) — Python. GUI / CLI |
> | Convert (C#, with built-in physics bake) | [mmd2gltf-cs](https://github.com/masaka1024/mmd2gltf-cs) — same output format as gui; bakes secondary motion with the in-house engine |
> | Run it in Unity | [mmd2gltf-unity-physics-importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer) — Editor extension. Bundled Bullet-compatible engine (no PhysX) |
> | Run it in Unreal Engine 5 | [mmd2gltf-ue5-physics-importer](https://github.com/masaka1024/mmd2gltf-ue5-physics-importer) — C++ plugin. C++ port of the same engine (no Chaos) |
> | Run it in Blender | `mmd2gltf-blender-physics-importer` (coming soon) — Add-on. Wires up Blender's built-in Bullet |
> | (Library) the physics engine itself | [mmd2gltf-cs-physics](https://github.com/masaka1024/mmd2gltf-cs-physics) — Bullet 2.75-compatible engine used by cs / Unity / UE5. Not used directly |
> | (Separate route) export glTF/FBX from Blender's mmd_tools | [mmd-to-gltf-exporter](https://github.com/masaka1024/mmd-to-gltf-exporter) — no `extras.mmd` (physics is not rebuilt) |
>
> Secondary-motion behavior matches between Unity and UE5 because they share the same engine. The Blender version drives Blender's built-in Bullet, so the motion is close but not identical.
>
> **This repository**: The **Python** converter (Windows EXE available). This is where the `.glb` is produced. Start here.
<!-- mmd2gltf-ecosystem:end -->

A tool that converts MMD PMX models (plus VMD motions) to **glTF 2.0 (.glb)**. Works as both a GUI and a CLI. It runs on the Python standard library alone; Pillow is used for texture conversion (BMP/TGA/sph/spa → PNG) and NumPy, optionally, for part of the alpha analysis.

Information that glTF cannot express (rigid bodies, joints, IK settings, toon/sphere material settings, and so on) is kept under `extras.mmd` as the original values. A receiving side (a game engine or another tool) can rebuild the MMD model structure from it.

> **▶ Windows:** a prebuilt EXE is available on the [Releases](https://github.com/masaka1024/mmd2gltf-gui/releases/latest) page (no Python required).

<p align="center">
  <img src="docs/screenshot.png" alt="mmd2gltf GUI" width="640">
</p>

## Screenshots

Tda-style Hatsune Miku Append / Hatsune Miku V4X (models by Tda), converted with this tool and imported with the [Unity importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer).

<p align="center">
  <img src="docs/screenshot_tda_unity.png" alt="Tda-style Hatsune Miku Append and V4X displayed in Unity" width="640">
</p>

> The model data itself is NOT included in this repository. Used under the [Piapro Character License](https://piapro.jp/license/pcl/summary) and each model's terms of use.
> © Crypton Future Media, INC. www.piapro.net

## Verified models

Conversion and display have been verified with the following models (obtain the model data from each distribution page):

| Model | Author | Main verification points |
|---|---|---|
| Tda-style Hatsune Miku Append Ver1.10 | Tda | Material colors & sphere maps, semi-transparent overlays (forehead hair shadow / cheek), toon reconstruction in Unity |
| Tda-style Hatsune Miku V4X Ver1.00 | Tda | Alpha classification of shared textures, render order of translucent materials (glasses / lenses) |
| IA (PMX model) | Omiya ([bowlroll](https://bowlroll.net/file/81272)) | Blend restoration of translucent hair (high-alpha), soft eyebrow / eye-lining translucency |

## Table of contents

- [Goal of this tool](#goal-of-this-tool)
- [Features](#features)
- [Requirements](#requirements)
- [Windows EXE (no Python needed)](#windows-exe-no-python-needed)
- [Installing the Python version](#installing-the-python-version)
- [Usage](#usage)
- [IK handling](#ik-handling)
- [Physics baking (experimental)](#physics-baking-experimental)
- [Inspecting rigid bodies visually](#inspecting-rigid-bodies-visually)
- [Option reference](#option-reference)
- [Viewer compatibility notes](#viewer-compatibility-notes)
- [What gets converted](#what-gets-converted)
- [Limitations](#limitations)
- [Tests](#tests)
- [Project layout](#project-layout)
- [License](#license)

## Goal of this tool

The purpose of mmd2gltf is to **carry MMD data into other environments**. glTF (.glb) is used as the container for that.

The output has a two-layer structure:

1. **Preservation layer (`extras.mmd`)** — rigid bodies, joints, IK settings, append parents, toon/sphere/edge material settings, and other information glTF cannot express, stored as the original PMX values. A receiving side can rebuild the model structure from this layer. For rigid bodies and joints, a converted view in glTF space and scale (`physicsGltf`) is written alongside.
2. **Display layer (standard glTF mesh, skin and animation)** — what ordinary glTF viewers show as-is. VMD motion is baked into keyframes with IK solved.

There is no reverse converter (glTF → PMX). The data layout is documented in `mmd2gltf/extrasMmd_schema.md` and `mmd2gltf/physicsGltf_schema.md`.

> **About usage terms:** The model's and motion's usage terms still apply after conversion. Because `extras.mmd` contains the original data, **distributing a converted file is equivalent to distributing the original data**. A model that prohibits redistribution cannot be distributed after conversion either.

### Unity importer

An editor extension that reads `extras.mmd` and rebuilds the model in Unity is published as a separate repository: **[mmd2gltf-unity-physics-importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer)** (requires UniGLTF / lilToon, for URP projects; README in Japanese). It rebuilds physics from the rigid-body and joint data, and restores an MMD-style toon look from the toon/sphere/outline material settings.

## Features

- **No extra libraries required** — runs on the standard library alone (Pillow for texture conversion, NumPy optionally for better alpha auto-detection).
- **PMX 2.0/2.1 parsing** — meshes, skinning, materials, morphs, and physics data.
- **VMD motion baking** — evaluates Bézier interpolation and bakes every frame in MMD's deformation order (deform hierarchy → append/grant → CCD-IK with axis limits).
- **IK control, 4 modes** — from full bake to "don't solve at all" (see [IK handling](#ik-handling)).
- **Information glTF can't express is kept under `extras.mmd`** — rigid bodies, joints, IK settings, append parents, and more.
- **GUI and CLI** — the GUI supports Japanese/English switching and drag & drop.
- **Prebuilt Windows EXE** — run without installing Python.

## Requirements

- **Windows EXE** — no Python installation needed.
- **Python version** — Python 3 (standard library only)
  - Optional: **Pillow** (needed when textures are not PNG/JPG)
  - Optional: **NumPy** (used by alpha auto-detection for pre-blending semi-transparent texels and UV-region sampling; conversion works without it)
  - Optional: **tkinterdnd2** (for drag & drop in the GUI)

## Windows EXE (no Python needed)

1. Download the latest zip from the [Releases](https://github.com/masaka1024/mmd2gltf-gui/releases) page.
2. Extract it and double-click `mmd2gltf.exe` to launch the GUI.
3. From there it's the same as the GUI described in [Usage](#usage). Pillow, NumPy and tkinterdnd2 are bundled.

> **About the first-launch warning**
> This is unsigned software from an individual developer, so Windows SmartScreen may show
> "Windows protected your PC" on first launch. Click "More info" → "Run anyway".
> Some antivirus software may flag PyInstaller-built executables; the source is available in this repository.

## Installing the Python version

Clone the repository and install the optional libraries as needed.

```bash
# Optional: when textures are not PNG/JPG
pip install Pillow

# Optional: better alpha auto-detection
pip install numpy

# Optional: drag & drop in the GUI
pip install tkinterdnd2
```

## Usage

### GUI

```bash
python gui.py
```

File selection and the main options (unlit, double-sided, morph mode, alpha mode, scale) are available on the main screen. "Advanced settings" exposes IK options, step, animation name, and physics-baking settings. Conversion runs in the background and the log is shown in the window.

- You can drag & drop files from Explorer onto the PMX/VMD fields (requires `tkinterdnd2`; the "Browse..." buttons work without it).
- The dropdown at the top right switches between Japanese and English (auto-selected from the OS locale on first launch).

### CLI

```bash
python -m mmd2gltf model.pmx -o model.glb
python -m mmd2gltf model.pmx --vmd motion.vmd -o dance.glb
```

### From Python

```python
from mmd2gltf import convert
```

## IK handling

When baking VMD motion you can choose from four ways to handle IK (legs, arms, etc.). The default full bake is usually fine; switch only when you see unintended foot motion.

| Mode | Flag | Behaviour |
| --- | --- | --- |
| **Full bake (default)** | `--vmd FILE` only | Solves all IK and bakes the result (30fps). IK on/off keys inside the VMD are respected. |
| **Don't solve IK** | `--no-ik` | Skips IK entirely and outputs the raw FK curves. |
| **Disable some** | `--disable-ik NAME` | Disables only IK bones whose name contains NAME (repeatable). E.g. `--disable-ik 足` for leg/toe IK. |
| **Ignore VMD IK keys** | `--ignore-vmd-ik` | Ignores IK on/off keys in the VMD. Useful for full-key motions where leg IK is keyed off and would otherwise not be solved. |

## Physics baking (experimental)

With `--bake-physics`, rigid-body physics for hair, skirts and similar parts is solved by a simplified simulation (PBD spring model) and the result is baked into bone keyframes. This is an approximation intended for glTF viewers that have no physics engine.

> **This feature is experimental and is not actively recommended at the moment.** The internal solver has not been updated since its early version and behaves quite differently from MMD itself (Bullet physics). If you want hair and skirts to move the way they do in MMD, rebuild the physics on the engine side (e.g. Unity) from the rigid-body and joint data stored in `extras.mmd`. Treat the baked result as a rough preview at most.

```bash
# Hair only (default)
python -m mmd2gltf model.pmx --vmd motion.vmd --bake-physics -o out.glb

# Everything including skirts
python -m mmd2gltf model.pmx --vmd motion.vmd --bake-physics --bake-target all -o out.glb
```

Physics baking does not modify the rigid-body and joint data in `extras.mmd`. There are many tuning parameters, but for the reason above this README does not document them individually; see `python -m mmd2gltf --help` if you need them. The GUI exposes the same settings under "Advanced settings".

## Inspecting rigid bodies visually

`tools/mmd_physics_inspector.html` is a standalone tool for opening a converted GLB in a browser and inspecting the contents of `extras.mmd` (just drag & drop the file onto the page; nothing to install).

It shows rigid bodies, joints and collision groups in 3D. Clicking a rigid body shows its shape, size, mode, bound bone and the groups it collides with. When building physics on the engine side, this lets you check where each rigid body is and whether it is really set to collide.

> **About the 16-bit field:** PMX rigid bodies have a 16-bit field called the "non-collision group flag", but **a set bit means the body DOES collide with that group** — it is Bullet's collision filter mask verbatim. PMXEditor displays it inverted (checked = no collision), so reading the field by its name gives the opposite result. This tool and the inspector follow the mask interpretation (the value written to `extras.mmd` is still the raw PMX one — see `mmd2gltf/physicsGltf_schema.md`).

## Option reference

| Option | Description |
| --- | --- |
| `--vmd FILE` | Bake a VMD motion as a glTF animation (IK solved, 30fps) |
| `--no-ik` | Don't solve any IK while baking (raw FK curves only) |
| `--disable-ik NAME` | Disable only IK bones whose name contains NAME (repeatable; e.g. `--disable-ik 足` for leg/toe IK) |
| `--ignore-vmd-ik` | Ignore IK on/off keys in the VMD (respected by default) |
| `--step N` | Sample every N-th frame to reduce file size (default 1 = all frames) |
| `--unlit` | Tag materials with `KHR_materials_unlit` (closer to MMD's toon look) |
| `--no-extras` | Omit `extras.mmd` |
| `--anim-name NAME` | Animation name |
| `--morph-mode MODE` | Morph encoding: `sparse` = compact (default), `dense` = maximum compatibility (use when the face breaks in viewers without sparse support), `none` = no morphs |
| `--alpha-mode MODE` | `auto` (default) analyses each texture's alpha distribution and picks OPAQUE/MASK/BLEND. `opaque`/`mask`/`blend` force one mode for all materials |
| `--force-double-sided` | Render all materials double-sided (when the back of hair or skirts disappears) |
| `--scale F` | Uniform scale from MMD units to glTF units/metres (default 0.08). MMD models are conventionally built at about 1 unit ≈ 8 cm, so `1.0` (no scaling) shows roughly 12.5× too large in glTF viewers. Applied to vertices, bone positions, SDEF parameters, morphs and baked translations; raw data in `extras.mmd` is untouched (the factor is recorded in `extras.mmd.unitScale`) |
| `--no-custom-attrs` | Omit MMD-specific vertex attributes (`_SDEF_C`/`_SDEF_R0`/`_SDEF_R1`/`_ADDUV1..4`/`_EDGESCALE`/`_WEIGHTTYPE`). Use this if Blender's standard glTF importer fails on them (the data remains under `extras.mmd`) |
| `--bake-physics` and related | Physics baking ([experimental](#physics-baking-experimental)); see `--help` for the full list |

## Viewer compatibility notes

- macOS Quick Look / Preview (RealityKit) doesn't support glTF morph targets at all. Morphs "missing" there is a viewer limitation; the file itself is fine.
- Viewers/loaders with incomplete sparse-accessor support may break the mesh when morphs are applied (see-through face, lip sync deforming the wrong place, etc.). In that case convert with `--morph-mode dense` (larger file). `dense` is also recommended for UniGLTF/UniVRM-based Unity importers.
- Verified viewers: three.js-based (gltf-viewer.donmccurdy.com), Babylon.js Sandbox (sandbox.babylonjs.com), and Blender 3.x+ glTF importer.

## What gets converted

Expressed directly in glTF:

- Meshes (positions, normals, UVs, one primitive per material)
- Skinning (BDEF1/2/4; SDEF/QDEF approximated as linear blends, with the original parameters preserved)
- Bone hierarchy (PMX bone order = skin.joints order, so indices are compatible)
- Vertex morphs → morph targets (sparse accessors), UV morphs → `TEXCOORD_0` targets, group morphs → expanded into composed targets
- Materials (diffuse → baseColor, double-sided flag, BLEND detection from alpha, embedded textures)
- VMD motion: Bézier interpolation evaluated and baked every frame in MMD's deformation order (deform hierarchy → append/grant → CCD-IK with axis limits). Morph keys become a weights animation

Stored under `extras.mmd` (raw PMX values, MMD left-handed coordinates):

- Rigid bodies and joints (physics), IK settings, append parents, fixed/local axes, display frames
- Bone morphs, material morphs, flip/impulse morphs
- Material sphere map / toon / edge / ambient / specular settings, memo
- Alpha classification (`alphaClass`) and the texture before pre-blending (`origTexture`)
- Per-vertex data kept as custom attributes: `_ADDUV1..4`, `_EDGESCALE`, `_SDEF_C/_SDEF_R0/_SDEF_R1`, `_WEIGHTTYPE`

`extras.mmd.physicsGltf` holds a view of the rigid bodies and joints converted to glTF space, scaled and expressed in bone-local terms (the raw data is left untouched). Use it when building physics on the engine side to avoid doing the coordinate conversion yourself.

Coordinate conversion: positions/normals `(x,y,z)→(x,y,-z)`, quaternions `(x,y,z,w)→(-x,-y,z,w)`, triangle winding flipped.

## Limitations

- PMD (legacy format) and PMX 2.1 soft bodies are not supported (convert to PMX with PMXEditor etc.)
- Physics baking is an approximation by a simplified simulation and does not reproduce MMD's Bullet physics (see [Physics baking (experimental)](#physics-baking-experimental))
- MMD's toon shading / sphere maps / edge rendering cannot be reproduced in glTF PBR, so the look depends on the viewer (`--unlit` gets closer)
- Shared toon textures (toon01–10.bmp) ship with MMD itself and are not embedded (the index is kept in extras)
- VMD camera, light and self-shadow keys are not handled

## Tests

```bash
python tests/make_test_data.py                # generate synthetic PMX/VMD
python -m mmd2gltf tests/test.pmx --vmd tests/test.vmd -o tests/test.glb
python tests/check_glb.py tests/test.glb      # structural check
```

## Project layout

```
mmd2gltf/
  pmx.py                  PMX 2.0/2.1 parser
  vmd.py                  VMD parser + Bézier interpolation
  animation.py            MMD deformation pipeline (append, CCD-IK) and baking
  bake_hair.py            Physics baking (experimental)
  drape.py                For physics baking: measures drape depth from the PMX mesh
  physics.py              Builds extras.mmd.physicsGltf (glTF-space view of rigid bodies/joints)
  mathutil.py             Quaternion / vector helpers
  gltf.py                 GLB builder (sparse accessor support)
  convert.py              Conversion core
  cli.py                  CLI
  extrasMmd_schema.md     extras.mmd specification
  physicsGltf_schema.md   extras.mmd.physicsGltf specification
gui.py                    GUI
tools/
  mmd_physics_inspector.html  Visual inspector for rigid bodies, joints and collision groups
  build_release.ps1           Builds the Windows EXE and the release zip
  release_pack.py             Checks the release contents and assembles the zip (called by the above)
tests/                    Synthetic test data generation and structural check
dist_docs/                Documents bundled with the EXE release
```

## License

The source code of this tool (mmd2gltf) is released under the MIT License. See [LICENSE](LICENSE) for details.

The Windows EXE bundles open-source components such as Python, Pillow, NumPy, tkinterdnd2, tkDnD, and Tcl/Tk. Their licenses are compiled in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
