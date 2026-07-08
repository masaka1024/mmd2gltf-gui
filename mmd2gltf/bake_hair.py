# -*- coding: utf-8 -*-
"""剛体ベイク（PBD）。physicsGltf からチェーンを抽出し、体モーション下で
髪の揺れをシミュレートして髪ボーンの回転キーを生成する。

このファイルは段階実装中。まずは chain 抽出まで。
依存: 標準ライブラリのみ（physics.py のベクトル演算を流用）。
"""
import math
# 本体(パッケージ)では from .physics import に変更すること
from .physics import (q_mul, q_rotate_vec, q_conj,
                     compute_bone_world_matrices, trs_to_mat, mat_mul,
                     mat_ident, mat_to_trs)

def _sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def _len(a): return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

class Particle:
    __slots__ = ("rb", "bone", "mass", "inv_mass", "parent", "rest_len",
                 "rest_dir_local", "ang_min", "ang_max", "rest_pos", "kinematic")
    def __init__(self, rb, bone, mass, kinematic):
        self.rb = rb              # physicsGltf.rigidBodies のindex
        self.bone = bone          # node index
        self.mass = mass
        self.inv_mass = 0.0 if (kinematic or mass <= 0) else 1.0/mass
        self.kinematic = kinematic
        self.parent = -1          # 親パーティクルのindex（chain内）
        self.rest_len = 0.0       # 親との静止距離
        self.ang_min = None       # jointの角度制限（ラジアン, x,y,z）
        self.ang_max = None
        self.rest_pos = None      # rest世界座標(glTF)

class Chain:
    def __init__(self):
        self.particles = []       # 親→子順に並んだ Particle
    def __len__(self): return len(self.particles)


def _bone_world_pos(bwm, bi):
    m = bwm[bi]
    return (m[0][3], m[1][3], m[2][3])


def extract_chains(physics_gltf, bone_world_matrices, only_names=None):
    """physicsGltf + node世界行列 から Chain のリストを構築。

    only_names: Noneなら全dynamic剛体。文字列リストを渡すと、剛体名にその
                いずれかを含むものだけ対象（例: ["髪"]）。
    """
    rbs = physics_gltf["rigidBodies"]
    jts = physics_gltf["joints"]
    bwm = bone_world_matrices

    def target(rb):
        if only_names is None:
            return True
        return any(k in rb["name"] for k in only_names)

    # joint から親子関係を作る: B の親は A
    parent_rb = {}      # rb_index -> (parent_rb_index, joint)
    for j in jts:
        parent_rb[j["rigidB"]] = (j["rigidA"], j)

    # dynamic（mode1/2）で対象の剛体をパーティクル化
    idx_of = {}
    parts = {}
    for i, rb in enumerate(rbs):
        if rb["mode"] in (1, 2) and target(rb):
            p = Particle(i, rb["bone"], rb["mass"], kinematic=False)
            p.rest_pos = _bone_world_pos(bwm, rb["bone"]) if 0 <= rb["bone"] < len(bwm) else None
            parts[i] = p

    # 親リンク・rest長・角度制限を埋める（反復中に parts へ追加するのでスナップショット）
    for i, p in list(parts.items()):
        link = parent_rb.get(i)
        if not link:
            continue
        pa_rb, j = link
        p.ang_min = j["angularLimitMin"]
        p.ang_max = j["angularLimitMax"]
        if pa_rb in parts:
            p.parent = pa_rb
            if p.rest_pos and parts[pa_rb].rest_pos:
                p.rest_len = _len(_sub(p.rest_pos, parts[pa_rb].rest_pos))
        else:
            # 親が dynamic でない = アンカー（頭など mode0 剛体）
            anchor_rb = rbs[pa_rb] if 0 <= pa_rb < len(rbs) else None
            if anchor_rb is not None:
                ap = Particle(pa_rb, anchor_rb["bone"], anchor_rb["mass"], kinematic=True)
                ap.rest_pos = _bone_world_pos(bwm, anchor_rb["bone"]) if 0 <= anchor_rb["bone"] < len(bwm) else None
                parts[pa_rb] = ap        # アンカーを追加
                p.parent = pa_rb
                if p.rest_pos and ap.rest_pos:
                    p.rest_len = _len(_sub(p.rest_pos, ap.rest_pos))

    # チェーンに分解: 各 dynamic パーティクルの「チェーンの根」でグループ化。
    # 根 = 親を辿って、親が kinematic(アンカー) か無い所まで登った最上位の dynamic。
    def chain_root(i):
        seen = 0
        while parts[i].parent != -1 and not parts[parts[i].parent].kinematic:
            i = parts[i].parent
            seen += 1
            if seen > 1000: break
        return i

    def depth_from_root(i):
        d = 0; seen = 0
        while parts[i].parent != -1 and not parts[parts[i].parent].kinematic:
            i = parts[i].parent; d += 1; seen += 1
            if seen > 500: break
        return d

    # アンカー(kinematic)に辿り着けるか判定。辿り着けない＝リング等の
    # 非チェーン構造とみなして除外する（スカートのリングはここで落ちる）。
    def reaches_anchor(i):
        seen = 0
        while True:
            pr = parts[i].parent
            if pr == -1:
                return False
            if parts[pr].kinematic:
                return True
            i = pr
            seen += 1
            if seen > 500:
                return False   # サイクル = 到達不能扱い
    excluded = set()
    for i, p in parts.items():
        if not p.kinematic and not reaches_anchor(i):
            excluded.add(i)

    groups = {}
    for i, p in parts.items():
        if p.kinematic or i in excluded:
            continue
        groups.setdefault(chain_root(i), []).append(i)

    chains = []
    for r, members in groups.items():
        members.sort(key=depth_from_root)
        ch = Chain()
        anchor_i = parts[members[0]].parent   # 根の親 = アンカー
        if anchor_i in parts and parts[anchor_i].kinematic:
            ch.particles.append(parts[anchor_i])
        for i in members:
            ch.particles.append(parts[i])
        chains.append(ch)
    # 長い順に
    chains.sort(key=lambda c: -len(c))
    # excluded を parts に添付（呼び出し側のログ用）
    return chains, parts, excluded


# ======================================================================
# PBD シミュレーション
# ======================================================================
def _add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def _scale(a, s): return (a[0]*s, a[1]*s, a[2]*s)
def _dot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def _norm(a):
    l = _len(a)
    return (a[0]/l, a[1]/l, a[2]/l) if l > 1e-12 else (0.0, 0.0, 0.0)
def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


class PBDState:
    """チェーン群の PBD 状態。pos/prev をパーティクル単位で持つ。"""
    def __init__(self, chains):
        self.chains = chains
        # 全パーティクルを一意リストに（重複アンカーは1つに）
        self.pos = {}     # rb_index -> (x,y,z)
        self.prev = {}
        self.part = {}    # rb_index -> Particle
        for ch in chains:
            for p in ch.particles:
                if p.rb not in self.part:
                    self.part[p.rb] = p
                    self.pos[p.rb] = p.rest_pos
                    self.prev[p.rb] = p.rest_pos

    def set_anchor(self, rb_index, world_pos):
        self.pos[rb_index] = world_pos
        self.prev[rb_index] = world_pos   # アンカーは速度を持たせない


def q_from_to(a, b):
    """単位ベクトル a を b に向ける最短回転クォータニオン。"""
    a = _norm(a); b = _norm(b); d = _dot(a, b)
    if d > 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    if d < -0.999999:
        ax = _cross((1.0, 0.0, 0.0), a)
        if _len(ax) < 1e-6:
            ax = _cross((0.0, 1.0, 0.0), a)
        ax = _norm(ax)
        return (ax[0], ax[1], ax[2], 0.0)
    ax = _cross(a, b)
    q = (ax[0], ax[1], ax[2], 1.0 + d)
    n = math.sqrt(sum(c*c for c in q))
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)


class SpringState:
    """スプリングボーン方式の状態。パーティクル=各ボーンの位置(tail)。"""
    def __init__(self, chains):
        self.chains = chains
        self.pos = {}       # rb -> world pos
        self.prev = {}
        self.part = {}
        self.rest_dir = {}  # rb(child) -> 親からのrest方向(world, 単位)。親rest回転=identity前提
        for ch in chains:
            for p in ch.particles:
                if p.rb not in self.part:
                    self.part[p.rb] = p
                    self.pos[p.rb] = p.rest_pos
                    self.prev[p.rb] = p.rest_pos
            # rest方向を各子で計算
            plist = ch.particles
            for k in range(1, len(plist)):
                c = plist[k]; pa = plist[k-1]
                self.rest_dir[c.rb] = _norm(_sub(c.rest_pos, pa.rest_pos))

    def set_anchor(self, rb, world_pos, world_rot=(0.0, 0.0, 0.0, 1.0)):
        self.pos[rb] = world_pos
        self.prev[rb] = world_pos
        self._anchor_rot = getattr(self, "_anchor_rot", {})
        self._anchor_rot[rb] = world_rot


def simulate_step(state, gravity_dir, dt, drag_force, stiffness_force,
                  gravity_power=0.0):
    """UniVRM風スプリングボーン1ステップ。set_anchor 済み前提。
      drag_force     : 速度損失率(0..1)。0.4なら60%の速度を保持
      stiffness_force: rest方向へ戻す nudge の強さ（*dt でスケール）
      gravity_power  : 重力の強さ（gravity_dir 方向、*dt でスケール）
    戻り値: seg_rot[rb] = 各ボーンセグメントのワールド回転(quat)
    """
    pos, prev, part, rest_dir = state.pos, state.prev, state.part, state.rest_dir
    anchor_rot = getattr(state, "_anchor_rot", {})
    seg_rot = {}
    stiff = stiffness_force * dt
    grav = _scale(_norm(gravity_dir), gravity_power * dt) if gravity_power else (0.0, 0.0, 0.0)

    for ch in state.chains:
        plist = ch.particles
        q_par = anchor_rot.get(plist[0].rb, (0.0, 0.0, 0.0, 1.0)) if plist[0].kinematic \
            else (0.0, 0.0, 0.0, 1.0)
        for k in range(1, len(plist)):
            c = plist[k]; pa = plist[k-1]
            rdir = rest_dir[c.rb]
            # rest方向を親の現回転で回した「あるべき向き」
            tgt_dir = _qrot(q_par, rdir)
            if c.inv_mass <= 0:
                cur_dir = _norm(_sub(pos[c.rb], pos[pa.rb]))
                aim = q_from_to(tgt_dir, cur_dir)
                seg_rot[c.rb] = q_mul(aim, q_par)
                q_par = seg_rot[c.rb]
                continue
            cur = pos[c.rb]
            # 慣性（速度保持）
            inertia = _scale(_sub(cur, prev[c.rb]), 1.0 - drag_force)
            # rest方向への有界 nudge ＋ 重力
            nxt = _add(cur, inertia)
            nxt = _add(nxt, _scale(tgt_dir, stiff))
            nxt = _add(nxt, grav)
            # 長さ拘束（親から rest_len）
            d = _sub(nxt, pos[pa.rb]); dl = _len(d)
            if dl > 1e-9:
                nxt = _add(pos[pa.rb], _scale(d, c.rest_len / dl))
            # 角度制限（rest方向 tgt_dir からの逸脱を amax 以内にコーンクランプ）
            cur_dir = _norm(_sub(nxt, pos[pa.rb]))
            if c.ang_max:
                amax = max(abs(c.ang_max[0]), abs(c.ang_max[2]))
                if amax > 1e-4:
                    cang = max(-1.0, min(1.0, _dot(tgt_dir, cur_dir)))
                    if cang < math.cos(amax):
                        axis = _cross(tgt_dir, cur_dir)
                        if _len(axis) > 1e-9:
                            axis = _norm(axis)
                            cur_dir = _rotate_about(tgt_dir, axis, amax)
                            nxt = _add(pos[pa.rb], _scale(cur_dir, c.rest_len))
                            # クランプ時は速度を少し殺して弾みを抑える
                            cur = _add(cur, _scale(_sub(nxt, cur), 0.5))
            prev[c.rb] = cur
            pos[c.rb] = nxt
            aim = q_from_to(tgt_dir, cur_dir)   # rest追従方向からの小偏差(≤amax)
            seg_rot[c.rb] = q_mul(aim, q_par)    # 親の完全回転(捻り保存)に偏差を合成
            q_par = seg_rot[c.rb]
    return seg_rot


def _qmul(a, b):
    return q_mul(a, b)

def _qrot(q, v):
    return q_rotate_vec(q, v)


def _rotate_about(v, axis, ang):
    """Rodrigues: v を axis(単位) 周りに ang 回転。"""
    c = math.cos(ang); s = math.sin(ang)
    return _add(_add(_scale(v, c), _scale(_cross(axis, v), s)),
                _scale(axis, _dot(axis, v) * (1 - c)))


# ======================================================================
# 出力: seg_rot(ワールド) -> ボーンローカル回転キー
# ======================================================================
def q_inv(q):
    return q_conj(q)  # 単位quat前提


def seg_rot_to_local(state, seg_rot):
    """各動的パーティクルの seg_rot(ワールド) を、
    ボーンローカル回転(親ボーン基準)に変換して返す。
    親rest回転=identity 前提（このPMXの骨は並進のみ）なので、
    local = inv(parent_seg_world) * this_seg_world。
    root(親がアンカー)の場合は parent_seg = アンカーのワールド回転。
    """
    anchor_rot = getattr(state, "_anchor_rot", {})
    out = {}  # bone(node index) -> quat(local)
    for ch in state.chains:
        plist = ch.particles
        for k in range(1, len(plist)):
            c = plist[k]; pa = plist[k-1]
            this_w = seg_rot.get(c.rb, (0.0, 0.0, 0.0, 1.0))
            if pa.kinematic:
                par_w = anchor_rot.get(pa.rb, (0.0, 0.0, 0.0, 1.0))
            else:
                par_w = seg_rot.get(pa.rb, (0.0, 0.0, 0.0, 1.0))
            out[c.bone] = q_mul(q_inv(par_w), this_w)
    return out


# ======================================================================
# 全フレーム・ベイク: 体アニメ下で髪を揺らし、髪ボーンの回転キーを生成
# ======================================================================
def bake_hair_rotations(chains, frame_anchor_fn, num_frames,
                        drag_force=0.85, stiffness_force=1.5, gravity_power=0.02,
                        gravity_dir=(0.0, -1.0, 0.0), dt=1.0/30.0,
                        warmup=20, substeps=1):
    """
    frame_anchor_fn(frame) -> {anchor_rb: (world_pos, world_rot_quat)}
        各フレームでのアンカー（頭など mode0 剛体）のワールド姿勢を返す関数。
    返り値: { bone_node_index: [ (frame, quat_xyzw), ... ] }
    """
    state = SpringState(chains)
    # ウォームアップ: 0フレーム目の姿勢で数十ステップ回して安定させる
    a0 = frame_anchor_fn(0)
    for _ in range(warmup):
        for rb, (wp, wr) in a0.items():
            state.set_anchor(rb, wp, wr)
        simulate_step(state, gravity_dir, dt, drag_force, stiffness_force, gravity_power)

    keys = {}
    for f in range(num_frames):
        a = frame_anchor_fn(f)
        for _ in range(substeps):
            for rb, (wp, wr) in a.items():
                state.set_anchor(rb, wp, wr)
            seg = simulate_step(state, gravity_dir, dt / substeps,
                                drag_force, stiffness_force, gravity_power)
        loc = seg_rot_to_local(state, seg)
        for bone, q in loc.items():
            keys.setdefault(bone, []).append((f, q))
    return keys


# ======================================================================
# convert.py 統合用: baked(体アニメ) から髪回転キーを生成
# ======================================================================
def _cquat(q):
    return (-q[0], -q[1], q[2], q[3])        # convert.py の cquat と同一

def _cpos(t, s):
    return (t[0] * s, t[1] * s, -t[2] * s)   # convert.py の cpos と同一


def bake_hair_into_gltf(gltf_json, baked, num_frames, physics_gltf, scale,
                        drag_force=0.85, stiffness_force=1.5, gravity_power=0.02,
                        gravity_dir=(0.0, -1.0, 0.0), fps=30.0,
                        only_names=("髪",)):
    """体アニメ baked を駆動源に、髪ボーンのローカル回転キーを生成する。

    gltf_json    : 組み上がった g.j（nodes 必須）
    baked        : {node_index: {"r":[MMD quat...], "t":[MMD vec3...]}}（bake()の戻り）
    num_frames   : len(times)
    physics_gltf : extras.mmd.physicsGltf（build_physics_gltf の出力）
    scale        : unitScale
    戻り値       : { node_index: [ (x,y,z,w), ... num_frames ] }（glTF局所回転）
    """
    nodes = gltf_json["nodes"]
    bwm = compute_bone_world_matrices(gltf_json)
    chains, parts, excluded = extract_chains(physics_gltf, bwm, only_names=only_names)

    # 親マップ
    parent = [-1] * len(nodes)
    for i, nd in enumerate(nodes):
        for c in nd.get("children", []):
            parent[c] = i

    # アンカー（kinematic）ボーンごとに root→bone のFKチェーンを用意
    anchors = {}   # rb -> (bone_index, [root..bone])
    for ch in chains:
        p0 = ch.particles[0]
        if p0.kinematic:
            b = p0.bone
            path = []
            bb = b
            while bb != -1:
                path.append(bb); bb = parent[bb]
            path.reverse()
            anchors[p0.rb] = (b, path)

    def local_mat_at(bi, f):
        d = baked.get(bi)
        if d and d.get("r"):
            q = _cquat(d["r"][f])
        else:
            q = nodes[bi].get("rotation", [0.0, 0.0, 0.0, 1.0])
        if d and d.get("t"):
            t = _cpos(d["t"][f], scale)
        else:
            t = nodes[bi].get("translation", [0.0, 0.0, 0.0])
        return trs_to_mat(t, q)

    def world_at(path, f):
        M = mat_ident()
        for bi in path:
            M = mat_mul(M, local_mat_at(bi, f))
        return M

    def anchor_fn(f):
        out = {}
        for rb, (b, path) in anchors.items():
            M = world_at(path, f)
            _, q = mat_to_trs(M)
            out[rb] = ((M[0][3], M[1][3], M[2][3]), q)
        return out

    keys = bake_hair_rotations(chains, anchor_fn, num_frames,
                               drag_force=drag_force, stiffness_force=stiffness_force,
                               gravity_power=gravity_power, gravity_dir=gravity_dir,
                               dt=1.0 / fps)
    # {bone: [(f,quat)...]} -> {bone: [quat...]}（フレーム順）
    out = {}
    for bone, ks in keys.items():
        ks.sort(key=lambda x: x[0])
        out[bone] = [q for _, q in ks]
    return out, len(excluded)
