# `extras.mmd.physicsGltf` Schema

[日本語](physicsGltf_schema.md) | English

> If this English translation and the Japanese original ever disagree,
> the Japanese original ([physicsGltf_schema.md](physicsGltf_schema.md)) takes precedence.

mmd2gltf keeps MMD (PMX) rigid body and joint information in `extras.mmd` of the glTF output.
Within it, **`physicsGltf`** is a view that has been **converted into a ready-to-use form**
for anyone building physics in a later stage (Unity, Unreal Engine, etc.).
You do not need to convert the coordinate system, units, or orientation yourself.

> glTF has no runtime physics. mmd2gltf does not bake the "sway" of hair or skirts;
> instead it keeps the physics definitions as this `physicsGltf`. Build the swaying parts on the engine side.
> (If you want to bake them into keyframes, use the `--bake-physics` option.)

---

## Relationship to the raw data

`extras.mmd` contains two kinds of data side by side. Use whichever suits your purpose.

| Key | Coordinate system | Purpose |
|---|---|---|
| `rigidBodies` / `joints` | **MMD left-handed, unconverted, unscaled** (see `coordinateNote`) | Complete preservation of the original. For those who need a faithful reconstruction |
| **`physicsGltf`** | **glTF system, scaled, bone-local** | For those building physics in an engine (recommended) |

Both refer to the same rigid bodies and joints. `physicsGltf` never overwrites the raw data; it is only added alongside it.

---

## Top level

```jsonc
"extras": { "mmd": {
  "physicsGltf": {
    "space": "boneLocal",       // default reference space
    "unitScale": 0.08,          // scale that has been applied (for reference)
    "eulerOrderSource": "YXZ",  // Euler order of the original PMX rigid body rotations (for auditing)
    "note": "...",              // description of the coordinate system and fallbacks
    "rigidBodies": [ ... ],
    "joints": [ ... ]
  }
}}
```

- **`space`** … the overall default. Each entry also has its own `space`, which takes precedence.
- **All angles are in radians**, **all distances are in glTF scene units** (`unitScale` applied).

---

## `rigidBodies[]`

```jsonc
{
  "name": "左後髪1＿1",
  "bone": 170,              // node index of the associated bone (-1 = no bone)
  "space": "boneLocal",     // "boneLocal" | "world"
  "shape": 2,               // 0=sphere / 1=box / 2=capsule
  "mode": 2,                // 0=follow bone / 1=physics / 2=physics + bone
  "group": 4,               // collision group 0..15
  "noCollisionMask": 65507, // collision group bitmask (16 bits). The PMX value is kept as-is
                            // [IMPORTANT] The key name comes from the PMX field name
                            // "non-collision group flag", but a set bit means "collides with that group"
                            // (the same as Bullet's collision filter mask; PMX Editor's checkbox display,
                            // where checked = no collision, is the inverse). Collision is checked in both
                            // directions: two bodies collide only when
                            // (1<<A.group & B.mask) && (1<<B.group & A.mask).
                            // Corresponding names in other implementations: three.js mmd-parser = groupTarget,
                            // saba = m_collisionGroup. The key name is kept unchanged for compatibility.
  "size": [0.0061, 0.0305, 0.0],          // depends on shape (below)
  "position": [0.0131, 0.0078, -0.0008],  // position relative to space
  "rotation": [-0.0218, 0.0123, -0.4922, 0.8701], // quat (x,y,z,w) relative to space
  "mass": 10.0,
  "linearDamping": 1.0,     // linear damping
  "angularDamping": 1.0,    // angular damping
  "restitution": 0.0,       // restitution (bounciness)
  "friction": 0.0
}
```

### `space`
- `"boneLocal"` … `position`/`rotation` are in **the local space of `bone`**.
  Create a child object of that bone in the engine and assign them directly to `localPosition`/`localRotation`.
- `"world"` … fallback for rigid bodies with `bone == -1`. The world space of the glTF scene.

### Meaning of `shape` and `size`
| shape | Shape | Meaning of size |
|---|---|---|
| 0 | Sphere | `size[0]` = radius (`size[1]`, `[2]` unused) |
| 1 | Box | `[hx, hy, hz]` = **half extents** (along each local axis) |
| 2 | Capsule | `size[0]` = radius, `size[1]` = height, `size[2]` unused. **Long axis = local Y** |

### `mode`
| mode | Behavior | Handling in the engine |
|---|---|---|
| 0 | Follow bone (no physics simulation) | Kinematic. Fixed to the bone |
| 1 | Physics | Dynamic. Moved by physics and drives the bone |
| 2 | Physics + bone | Dynamic, but with correction toward the bone position |

---

## `joints[]`

All joints are `type: 0` (6DOF spring constraint). Each connects two rigid bodies, A and B.

```jsonc
{
  "name": "左後髪1＿2",
  "type": 0,
  "rigidA": 16,             // index into physicsGltf.rigidBodies[]
  "rigidB": 21,
  "space": "boneLocal",     // unified to the bone-local space of rigid body B (child side)
  "refBone": 171,           // node index of the reference bone (= rigid body B's bone; -1 for world)
  "position": [ ... ],      // relative to space
  "rotation": [ x,y,z,w ],  // relative to space
  "linearLimitMin": [0,0,0],      // linear limits (glTF units)
  "linearLimitMax": [0,0,0],
  "angularLimitMin": [-0.785, 0, -0.785],  // angular limits (radians)
  "angularLimitMax": [ 0.785, 0,  0.785],
  "springPosition": [0,0,0],      // linear spring constants (glTF units)
  "springRotation": [0,0,0]       // angular spring constants
}
```

- **`rigidA`/`rigidB`** … indices into the `physicsGltf.rigidBodies` array (same order as `rigidBodies`).
- **`angularLimitMin/Max`** … radians. Because of the mirroring (z flip) into the glTF system, the X/Y axes have their signs flipped and the Z axis is unchanged (already converted).
- **Axes whose limits are `min == max == 0`** … that axis is **locked (no degree of freedom)**.
  For example, the joint at the root of a chain has all axes at 0 and serves to anchor the hair.

---

## Building it in Unity (example)

1. Import the glTF/GLB and obtain the skeleton.
2. For each rigid body, find the `Transform` that corresponds to `bone` (node index).
3. Create a child GameObject under it and set `localPosition = position` / `localRotation = rotation`.
4. Add a Collider:
   - Sphere → `SphereCollider` (`radius = size[0]`)
   - Box → `BoxCollider` (`size = size * 2`; **mind the half-extent → full-size conversion**)
   - Capsule → `CapsuleCollider` (`radius = size[0]`, `height = size[1]`, `direction = Y-Axis`)
5. If `mode >= 1`, add a `Rigidbody`: `mass`, `drag = linearDamping`, `angularDrag = angularDamping`.
   For `mode == 0`, set `isKinematic = true`.
6. For joints, set up a `ConfigurableJoint` between `rigidA` ↔ `rigidB`.
   Convert angular limits from **radians to degrees** (`* 180/π`) and apply them to `angularXLimit`, etc.

> Note: Unity's `BoxCollider.size` is the full size, while the box `size` in `physicsGltf` is half extents. Multiply by 2.

## Building it in Unreal Engine (overview)

In the Skeletal Mesh's Physics Asset, add a Body (collider) to the bone that corresponds to each `bone`,
and set `shape`/`size`/`position`/`rotation`. For joints, connect `rigidA` ↔ `rigidB` with a Physics Constraint,
converting the angular limits to degrees.

---

## Reference: definition of the coordinate conversion (for auditing)

`physicsGltf` is the result of converting the raw data (`rigidBodies`/`joints`) with the following rules.

- Position: `(x, y, -z) × unitScale`
- Rotation: `euler(x, y, z radians, order=YXZ) → quat → (-x, -y, z, w)`
- Size: `× unitScale` (scalar dimensions, no sign flip)
- Angular limits: because of the z-flip mirroring, the X/Y axes are sign-flipped and min/max swapped; the Z axis is unchanged
- Bone-local conversion: `offset = inverse(boneWorldMatrix) × rigidWorldMatrix`

`eulerOrderSource: "YXZ"` is the order this tool has established. If you re-convert from the raw data yourself, use this order.
