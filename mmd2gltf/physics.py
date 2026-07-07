# -*- coding: utf-8 -*-
"""PMX剛体/ジョイントの raw値(extras.mmd)を glTF系のボーンローカルへ変換し、
extras.mmd.physicsGltf として併載するためのモジュール。

依存: 標準ライブラリのみ（numpy不要）。convert.py と同じピュアPython方針。

座標系（convert.py の cpos/cquat と一致）:
    位置 : (x, y, -z) * unitScale
    回転 : euler(x,y,z rad, order=YXZ) -> quat -> (-x,-y,z,w)
    サイズ: * unitScale（スカラー寸法、符号反転なし）
    角度制限: z反転の鏡映で X/Y軸は符号反転+min/max入替、Z軸は不変
オイラー順序 YXZ は実データ検証で確定（髪カプセル長軸ズレ 平均0.00°）。
"""
import math

MMD_EULER_ORDER = "YXZ"


# ----------------------------------------------------------------------
# quaternion  (x, y, z, w)
# ----------------------------------------------------------------------
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)

def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])

def _q_axis(ax, ang):
    s = math.sin(ang * 0.5)
    c = math.cos(ang * 0.5)
    return (ax[0]*s, ax[1]*s, ax[2]*s, c)

_AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}

def euler_to_quat(rx, ry, rz, order=MMD_EULER_ORDER):
    comp = {"X": _q_axis(_AXES["X"], rx),
            "Y": _q_axis(_AXES["Y"], ry),
            "Z": _q_axis(_AXES["Z"], rz)}
    q = (0.0, 0.0, 0.0, 1.0)
    for ax in order:                 # intrinsic: q = q1 * q2 * q3
        q = q_mul(q, comp[ax])
    return q

def q_normalize(q):
    n = math.sqrt(sum(c*c for c in q)) or 1.0
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)

def q_rotate_vec(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    r = q_mul(q_mul(q, qv), q_conj(q))
    return (r[0], r[1], r[2])


# ----------------------------------------------------------------------
# 4x4 matrix  (list of 4 lists of 4 floats)
# ----------------------------------------------------------------------
def mat_ident():
    return [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]

def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]

def mat_inv(m):
    """一般 4x4 逆行列（Gauss-Jordan）。"""
    n = 4
    a = [list(m[i]) + [1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-15:
            raise ValueError("singular matrix")
        a[col], a[piv] = a[piv], a[col]
        pv = a[col][col]
        a[col] = [x / pv for x in a[col]]
        for r in range(n):
            if r != col:
                f = a[r][col]
                if f:
                    a[r] = [a[r][k] - f * a[col][k] for k in range(2 * n)]
    return [row[n:] for row in a]

def trs_to_mat(t, q, s=(1.0, 1.0, 1.0)):
    x, y, z, w = q
    R = [[1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
         [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z-x*w)],
         [2*(x*z-y*w),     2*(y*z+x*w), 1-2*(x*x+y*y)]]
    M = mat_ident()
    for i in range(3):
        for j in range(3):
            M[i][j] = R[i][j] * s[j]
        M[i][3] = t[i]
    return M

def mat_to_trs(M):
    """4x4 -> (translation[3], quat[xyzw])。回転成分にスケール無し前提。"""
    t = (M[0][3], M[1][3], M[2][3])
    m00, m11, m22 = M[0][0], M[1][1], M[2][2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (M[2][1] - M[1][2]) / s
        y = (M[0][2] - M[2][0]) / s
        z = (M[1][0] - M[0][1]) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (M[2][1] - M[1][2]) / s
        x = 0.25 * s
        y = (M[0][1] + M[1][0]) / s
        z = (M[0][2] + M[2][0]) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (M[0][2] - M[2][0]) / s
        x = (M[0][1] + M[1][0]) / s
        y = 0.25 * s
        z = (M[1][2] + M[2][1]) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (M[1][0] - M[0][1]) / s
        x = (M[0][2] + M[2][0]) / s
        y = (M[1][2] + M[2][1]) / s
        z = 0.25 * s
    return t, q_normalize((x, y, z, w))


# ----------------------------------------------------------------------
# MMD -> glTF 基本変換
# ----------------------------------------------------------------------
def pos_mmd_to_gltf(pos, unit_scale):
    return (pos[0] * unit_scale, pos[1] * unit_scale, -pos[2] * unit_scale)

def quat_mmd_to_gltf(rot_euler_rad, order=MMD_EULER_ORDER):
    q = euler_to_quat(rot_euler_rad[0], rot_euler_rad[1], rot_euler_rad[2], order)
    return (-q[0], -q[1], q[2], q[3])

def size_mmd_to_gltf(size, unit_scale):
    return [c * unit_scale for c in size]

def rigid_world_matrix_gltf(rb, unit_scale):
    return trs_to_mat(pos_mmd_to_gltf(rb["pos"], unit_scale),
                      quat_mmd_to_gltf(rb["rot"]))


# ----------------------------------------------------------------------
# 組み上がった gltf json (builder.j) -> node index別 rest世界行列
# ----------------------------------------------------------------------
def compute_bone_world_matrices(gltf_json):
    nodes = gltf_json["nodes"]
    n = len(nodes)
    parent = [-1] * n
    for i, nd in enumerate(nodes):
        for c in nd.get("children", []):
            parent[c] = i

    def local(nd):
        if "matrix" in nd:                       # 列優先で来るので転置
            m = nd["matrix"]
            return [[m[j * 4 + i] for j in range(4)] for i in range(4)]
        t = nd.get("translation", [0.0, 0.0, 0.0])
        q = nd.get("rotation", [0.0, 0.0, 0.0, 1.0])
        s = nd.get("scale", [1.0, 1.0, 1.0])
        return trs_to_mat(t, q, s)

    world = [None] * n

    def wm(i):
        if world[i] is not None:
            return world[i]
        m = local(nodes[i])
        world[i] = m if parent[i] == -1 else mat_mul(wm(parent[i]), m)
        return world[i]

    for i in range(n):
        wm(i)
    return world


# ----------------------------------------------------------------------
# physicsGltf ビルダー
# ----------------------------------------------------------------------
def build_physics_gltf(mmd_extras, bone_world_matrices):
    """extras.mmd（raw）と node別世界行列から physicsGltf ブロックを生成。"""
    S = mmd_extras["unitScale"]
    rbs = mmd_extras.get("rigidBodies", [])
    jts = mmd_extras.get("joints", [])

    def bone_mat(bi):
        if 0 <= bi < len(bone_world_matrices):
            return bone_world_matrices[bi]
        return None

    out_rbs = []
    for rb in rbs:
        rigid_world = rigid_world_matrix_gltf(rb, S)
        bm = bone_mat(rb["bone"])
        if bm is not None:
            local = mat_mul(mat_inv(bm), rigid_world)
            space = "boneLocal"
        else:
            local = rigid_world
            space = "world"
        t, q = mat_to_trs(local)
        out_rbs.append({
            "name": rb["name"], "bone": rb["bone"], "space": space,
            "shape": rb["shape"], "mode": rb["mode"],
            "group": rb["group"], "noCollisionMask": rb["no_collision_mask"],
            "size": size_mmd_to_gltf(rb["size"], S),
            "position": list(t), "rotation": list(q),
            "mass": rb["mass"],
            "linearDamping": rb["linear_damping"],
            "angularDamping": rb["angular_damping"],
            "restitution": rb["restitution"], "friction": rb["friction"],
        })

    out_jts = []
    for j in jts:
        jw = trs_to_mat(pos_mmd_to_gltf(j["pos"], S), quat_mmd_to_gltf(j["rot"]))
        rb_b = rbs[j["rigid_b"]] if 0 <= j["rigid_b"] < len(rbs) else None
        bm = bone_mat(rb_b["bone"]) if rb_b is not None else None
        if bm is not None:
            local = mat_mul(mat_inv(bm), jw)
            space = "boneLocal"
            ref_bone = rb_b["bone"]
        else:
            local = jw
            space = "world"
            ref_bone = -1
        t, q = mat_to_trs(local)
        rmin, rmax = j["rot_min"], j["rot_max"]
        pmin, pmax = j["pos_min"], j["pos_max"]
        out_jts.append({
            "name": j["name"], "type": j["type"],
            "rigidA": j["rigid_a"], "rigidB": j["rigid_b"],
            "space": space, "refBone": ref_bone,
            "position": list(t), "rotation": list(q),
            "linearLimitMin": [pmin[0] * S, pmin[1] * S, -pmax[2] * S],
            "linearLimitMax": [pmax[0] * S, pmax[1] * S, -pmin[2] * S],
            "angularLimitMin": [-rmax[0], -rmax[1], rmin[2]],
            "angularLimitMax": [-rmin[0], -rmin[1], rmax[2]],
            "springPosition": [j["spring_pos"][0] * S, j["spring_pos"][1] * S,
                               j["spring_pos"][2] * S],
            "springRotation": list(j["spring_rot"]),
        })

    return {
        "space": "boneLocal",
        "unitScale": S,
        "eulerOrderSource": MMD_EULER_ORDER,
        "note": ("rigidBodies/joints are expressed in the local space of their "
                 "own bone (rigid) / rigidB's bone (joint). bone==-1 entries "
                 "fall back to glTF world space (see per-entry 'space'). "
                 "Angles in radians."),
        "rigidBodies": out_rbs,
        "joints": out_jts,
    }
