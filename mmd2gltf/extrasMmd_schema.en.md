# `extras.mmd` Schema

[日本語](extrasMmd_schema.md) | English

mmd2gltf stores MMD (PMX)-specific information that standard glTF cannot express,
together with the data needed to restore anything the conversion pipeline altered irreversibly,
in `extras.mmd` of the glTF JSON.

Importers for Unity, Unreal, and other engines can read this information
to rebuild the model's original MMD appearance and behavior.

> For `physicsGltf`, the converted view of rigid bodies and joints (physics),
> see the separate document **`physicsGltf_schema.en.md`**.
> This document covers every other field.

> **This file (together with its Japanese original) is the authoritative specification.**
> The copies kept in the importer repositories are stubs that point here.
> When the specification changes, always update this repository first; the importers
> follow by bumping the tag they reference. (Editing both places independently is
> what causes them to drift apart.)
>
> If this English translation and the Japanese original ever disagree,
> the Japanese original ([extrasMmd_schema.md](extrasMmd_schema.md)) takes precedence.

---

## Two categories of information in extras.mmd

| Category | Contents | Examples |
|---|---|---|
| (1) MMD-specific information that cannot be carried into glTF | Things with no corresponding concept in glTF's PBR materials or nodes. They are lost in conversion unless kept in extras | sphere, toon, edge, ambient/specular, rigid bodies, original morph data |
| (2) Data for restoring irreversible edits | When the pipeline modifies textures or alphaMode for display compatibility, the information needed to get back to the "before" state | `origTexture`, `alphaClass` |

### What does NOT go here

**Anything that falls into neither (1) nor (2) is not placed in `extras.mmd`.**
In particular, values that the receiver can compute from the GLB itself
(`meshes` / `skins` / `nodes` / `materials`, etc.) are not stored,
even if they would be useful for conversion or reproduction.

There are two reasons:

- `extras.mmd` is the **preservation layer for the original PMX data**. Mixing in values
  computed by the tool would force anyone writing a restoration tool to work out
  "which fields are original and which came from the tool."
- Calculation methods will keep improving. Baking a computed value into the GLB freezes
  the interpretation at that point in time, and previously exported GLBs would carry
  the old interpretation forever.

Example: how far a skirt's cloth sags (drape depth) can be derived from vertex positions,
skin weights, and bone positions and child directions, so it is a value the receiver
should measure, and it is not placed in `extras.mmd`. If implementations need to agree,
**we publish the calculation algorithm as a separate document instead of distributing the numbers.**

### Per-vertex data (supplementary note)

**Per-vertex MMD data** such as SDEF parameters (C/R0/R1), additional UVs, edge scale,
and weight type **is not stored in `extras.mmd`.**
It is exported as custom vertex attributes on the mesh
(`_SDEF_C` / `_SDEF_R0` / `_SDEF_R1` / `_ADDUV1..4` / `_EDGESCALE` / `_WEIGHTTYPE`).
The `_SDEF_*` values are already converted to the glTF coordinate system with `unitScale` applied.
`_WEIGHTTYPE` and `_SDEF_*` are exported only for models that contain SDEF vertices.

If `--no-custom-attrs` is specified at conversion time, these attributes are not exported
and this information does not remain in the GLB (they are exported by default).

---
## Where it is attached (4 places)

```
glTF root   .extras.mmd   … model info, original morphs, original rigid bodies/joints, physicsGltf
materials[i].extras.mmd   … MMD-specific material parameters + alpha restoration info
nodes[i]    .extras.mmd   … MMD-specific bone parameters (IK, inherit parent, etc.)
(baked animations and the like have no extras)
```

When `--no-extras` is specified (`extras=False`), none of these are exported.

---

## 1. Root `extras.mmd`

```jsonc
"extras": { "mmd": {
  "coordinateNote": "...",      // description of the coordinate system (see below)
  "unitScale": 0.08,            // scale that has been applied
  "format": "pmx",
  "version": 2.0,               // PMX version
  "name": "Tda式ミク・アペンド",  // model name (Japanese)
  "nameEn": "...",              // model name (English)
  "comment": "...",             // comment field (often contains the terms of use)
  "commentEn": "...",
  "morphs": [ ... ],            // original morph data (below)
  "displayFrames": [ ... ],     // display frames (PMX raw)
  "bones": [ ... ],             // basic bone info (PMX raw, below)
  "rigidBodies": [ ... ],       // original rigid bodies (PMX raw, unconverted)
  "joints": [ ... ],            // original joints (PMX raw, unconverted)
  "physicsGltf": { ... }        // converted physics view → physicsGltf_schema.en.md
}}
```

### Coordinate convention (coordinateNote)

The numbers under `extras.mmd` are **raw PMX values**.
They are kept in MMD's left-handed system (x, y, z) and are **unscaled**.
To bring them into glTF space, convert positions as (x, y, -z) and quaternions as (-x, -y, z, w),
then multiply by `unitScale`.

### morphs (original morph data)

```jsonc
{
  "name": "あ", "nameEn": "a",
  "panel": 3,                 // MMD display panel (1=eyebrow 2=eye 3=mouth 4=other)
  "type": 1,                  // PMX morph type (0=group 1=vertex 2=bone 3=UV 4-7=additional UV 8=material)
  "target": 5,                // glTF morph target index (vertex/UV morphs only; null otherwise)
  "offsets": [ ... ]          // when type is not 1 (vertex) / 3 (UV), the raw PMX offsets are kept
}
```

Vertex morphs and UV morphs are already converted into glTF morph targets in the main body.
Group, bone, and material morphs have no corresponding concept in glTF,
so their raw offsets are kept in `offsets` (the importer is expected to interpret them).

### bones (basic bone info)

```jsonc
"bones": [
  { "name": "しっぽ１３", "nameEn": "tail13", "flags": 8193, "parent": 165 }
]
```

- The order matches glTF `nodes[0 .. len(bones)-1]` (= PMX bone order), and
  the array index is the bone number itself (= the glTF node index).
- `flags` is the raw PMX value. From it, the receiver can determine things such as
  "is this bone translatable (0x0004)?" without guessing from whether it has a rigid body.
  Example: **a translation channel is legitimate only for translatable bones and for
  bones whose position is baked by the physics bake (bones with mode 1/2 rigid bodies).**
- The same `flags` are also in each bone node's `nodes[i].extras.mmd.flags`
  (→ Section 3). For receivers that want them as a list without walking the nodes,
  the same original values are also kept here (both are PMX raw and identical).
- Flag-dependent details such as IK definitions are on the `nodes[i].extras.mmd` side.

---

## 2. Material `materials[i].extras.mmd`

```jsonc
"extras": { "mmd": {
  // ---- (1) MMD-specific parameters not present in glTF ----
  "nameEn": "hairshadow",
  "ambient": [0.5, 0.5, 0.5],
  "specular": [0.0, 0.0, 0.0],
  "specularPower": 5.0,
  "flags": 0,                 // PMX drawing flags (bits for double-sided / ground shadow / self shadow / edge, etc.)
  "edgeColor": [0,0,0,1],
  "edgeSize": 1.0,
  "sphereMode": 0,            // 0=disabled 1=multiply (sph) 2=add (spa) 3=sub-texture
  "sphereTexture": -1,        // index into glTF textures[] (-1 = none)
  "toonTexture": -1,          // glTF texture index of an individual toon (-1 = none)
  "toonShared": 3,            // shared toon number, 0-based (0=toon01.bmp … 9=toon10.bmp, -1 = individual)
  "memo": "",                 // PMX memo field

  // ---- (2) restoration data for alpha edits ----
  "alphaClass": "blend",      // the material's "true" alpha category
  "origTexture": 14           // glTF index of the unmodified texture (-1 = the standard texture is unmodified)
}}
```

### alphaClass / origTexture in detail

> **Important:** the base color texture in the glTF may have been modified.
> For a faithful MMD look, decide how to draw each material from `alphaClass`,
> not from glTF's `alphaMode`, and use `origTexture` when it is present.

So that viewers that are weak at transparency sorting do not break, the pipeline
may **prebake** semi-transparent textures (composite them onto a background color
to make them opaque) and export them with alphaMode=MASK. This edit is irreversible,
so restoration information is kept for importers that can alpha-blend correctly.

| alphaClass | Meaning | Recommended importer behavior |
|---|---|---|
| `"opaque"` | Effectively opaque | Opaque is fine |
| `"mask"`   | Cutout (binary cut-outs such as hair strands) | Cutout by default (take the cutoff from glTF's `alphaCutoff`). If `origTexture` is present, drawing the unmodified version as **Transparent (with depth write, TwoPass, etc.)** reproduces soft eyebrows and see-through hair as well (recommended) |
| `"blend"`  | Actually needs semi-transparent blending | **Blend with the real alpha using `origTexture`** (Transparent) |

- When `origTexture >= 0`, the **unmodified texture** from before the prebake is bundled
  in the glTF under the name `orig_<original file name>`.
  Use it for faithful reproduction.
- When `origTexture == -1`, `baseColorTexture` is itself unmodified.
- The glTF `alphaMode` is a value that prioritizes viewer compatibility, while
  `alphaClass` says "how MMD actually intends it to be drawn." Use them accordingly.

### Mostly translucent materials (translucent)

Materials such as eyeglass lenses, which **contain opaque pixels but whose used UV area
is mostly (>50%) intermediate alpha**, are also detected by actually sampling the UV area
(measured on Tda式ミクV4X: lens = 77% translucent, mean alpha 0.76; frame megane = 46%).

In the standard GLB these materials **stay MASK + prebaked** (viewer safety first),
but their `alphaClass` becomes `"blend"`. Importers that support it should use
`origTexture` and alpha-blend with the real alpha in the transparent queue.

### Draw order (compositing in material index order)

MMD draws and composites **in material index order**, so model authors design
overlaps around that order. Importers should build a draw order that preserves
the material index order.

- When drawing a `"mask"` material as semi-transparent using `origTexture`, draw it
  **before** the true `"blend"` materials, **while writing depth**. Whether see-through hair
  and objects behind lenses look right is decided here.
- `"blend"` materials are composited in material index order.

Putting every material into the same transparent queue causes, through the combination
of depth writes and sub-mesh order, a bug where "materials with a later index behind
a semi-transparent surface disappear" (observed on Tda式ミクV4X).

> Concrete queue values for each engine are an implementation matter for the receiver.
> For values proven on Unity / lilToon, see `Assets/Editor/UnityMapping.md`
> in mmd2gltf-unity-physics-importer.

### Overlay materials (semi-transparent areas of a shared texture)

**Dedicated materials that use only the semi-transparent area of the face texture**,
such as the shadow of the bangs on the forehead (hairshadow) or cheek blush,
cannot be detected by per-texture classification, so they are judged by actually
sampling the UV area the material uses (95% or more of the used area has α<0.5 and the maximum α<0.6).

These materials are exported in the glTF from the start with
`alphaMode=BLEND` + `baseColorTexture=origTexture` (unmodified),
and their `alphaClass` is `"blend"`.
Importers can restore them correctly as-is by treating them as ordinary `"blend"`
(Transparent + real alpha).

---

## 3. Bone node `nodes[i].extras.mmd`

```jsonc
"extras": { "mmd": {
  "nameEn": "left elbow",
  "flags": 27,                 // PMX bone flags (bits for rotatable/translatable/IK/inherit/fixed axis, etc.)
  "layer": 0,                  // deformation layer

  // ---- the following exist only depending on the flags ----
  "tail_bone": 12,             // connected (tail) bone index (for display)
  "tail_offset": [0, 1.2, 0],  // or the tail offset
  "inherit_parent": 8,         // inherit parent bone index for rotation/translation inheritance
  "inherit_ratio": 0.5,        // inherit ratio
  "fixed_axis": [0,0,1],       // axis for fixed-axis bones
  "local_x": [...], "local_z": [...],  // local axes
  "external_key": 0,           // external parent key
  "ik": {                      // IK bones only
    "target": 34,              // IK target bone
    "loop": 40,                // iteration count
    "limit_angle": 1.0,        // unit angle (radians)
    "links": [                 // IK links (min/max only on links with angle limits)
      { "bone": 33, "min": [x,y,z], "max": [x,y,z] },
      { "bone": 32 }
    ]
  }
}}
```

Coordinate values are PMX raw, the same as the root `coordinateNote`.
With `--solve-ik` (ON by default), IK has already been used for baking the animation,
but the definitions themselves remain here for engines that want to rebuild IK at runtime.

---

## Versioning and compatibility

- The meaning and coordinate system of existing keys will not change (additions only).
- Importers should **ignore unknown keys**.
- New categories other than `"blend"` may be added to `alphaClass` in the future;
  treating unknown values as equivalent to `"mask"` is recommended.
- The name and version of the tool used for export are recorded in glTF's standard
  `asset.generator`. Importers that want to determine the spec version should read it there
  (`extras.mmd` is the preservation layer for the original PMX data, so the tool's version is not stored in it).
- The authoritative version of this specification is `mmd2gltf/extrasMmd_schema.md` in mmd2gltf-gui
  (this file is its English translation).
