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
    """スプリングボーン方式の状態。パーティクル=各ボーンの位置(tail)。

    rest_pos / rest_dir は形状の基準なので常に rest（変えない）。
    pos / prev の初期値だけ init_pos で上書きできる（開始ポーズからシード）。
    init_pos: {rb:(x,y,z)} または None。None or 未含有なら rest_pos でシード。
    """
    def __init__(self, chains, init_pos=None):
        self.chains = chains
        self.pos = {}       # rb -> world pos
        self.prev = {}
        self.part = {}
        self.rest_dir = {}  # rb(child) -> 親からのrest方向(world, 単位)。親rest回転=identity前提
        for ch in chains:
            for p in ch.particles:
                if p.rb not in self.part:
                    self.part[p.rb] = p
                    seed = init_pos.get(p.rb) if init_pos else None
                    seed = seed if seed is not None else p.rest_pos
                    self.pos[p.rb] = seed
                    self.prev[p.rb] = seed   # prev も同位置 = 初速ゼロ
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
    chains, parts, excluded, lateral = extract_chains_bfs(
        physics_gltf, bwm, only_names=only_names)

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

    def bone_path(bi):
        path = []
        bb = bi
        while bb != -1:
            path.append(bb); bb = parent[bb]
        path.reverse()
        return path

    def bone_world_pos(bi, f):
        M = world_at(bone_path(bi), f)
        return (M[0][3], M[1][3], M[2][3])

    # 下半身系(スカート等)判定用の腰Y（下半身→腰→センターの順で探す）
    _node_names = [nd.get("name", "") for nd in nodes]
    _waist = -1
    for _nm in ("下半身", "腰", "センター", "Center"):
        if _nm in _node_names:
            _waist = _node_names.index(_nm); break

    def anchor_fn(f):
        out = {}
        for rb, (b, path) in anchors.items():
            M = world_at(path, f)
            _, q = mat_to_trs(M)
            out[rb] = ((M[0][3], M[1][3], M[2][3]), q)
        return out

    # コライダー収集: mode0 のカプセル/球（脚・体など）。dynamic を押し出す障害物。
    rbs_pg = physics_gltf["rigidBodies"]
    colliders_def = []   # (shape, bone_path, size, local_pos, local_rot)
    for rb in rbs_pg:
        if rb.get("mode") == 0 and rb.get("shape") in (0, 2):  # 0=球 2=カプセル
            bi = rb.get("bone", -1)
            if not (0 <= bi < len(nodes)):
                continue
            path = []
            bb = bi
            while bb != -1:
                path.append(bb); bb = parent[bb]
            path.reverse()
            colliders_def.append((rb["shape"], path, rb["size"],
                                  rb["position"], rb["rotation"]))

    def colliders_at(f):
        cols = []
        for shape, path, size, lpos, lrot in colliders_def:
            W = world_at(path, f)
            M = mat_mul(W, trs_to_mat(lpos, lrot))
            c = (M[0][3], M[1][3], M[2][3])
            if shape == 0:               # 球
                cols.append(("sphere", c, size[0]))
            else:                        # カプセル（長軸=ローカルY）
                _, wq = mat_to_trs(M)
                ay = q_rotate_vec(wq, (0.0, 1.0, 0.0))
                half = size[1] * 0.5
                p0 = (c[0] - ay[0]*half, c[1] - ay[1]*half, c[2] - ay[2]*half)
                p1 = (c[0] + ay[0]*half, c[1] + ay[1]*half, c[2] + ay[2]*half)
                cols.append(("capsule", p0, p1, size[0]))
        return cols

    # 修正1: 初期位置シードは「下半身系(スカート等: アンカーが腰以下)」のみ
    #   1F目FK位置に。髪などは rest シードのまま（=ベースライン挙動を厳密維持。
    #   髪ボーンは凍結IK回転を含みうるので FK シードは形状を歪めるため）。
    def body_axis_at(f):
        if _waist < 0:
            return None
        return (bone_world_pos(_waist, f), (0.0, 1.0, 0.0))

    skirt_rbs = set()
    if _waist >= 0:
        _wy = bone_world_pos(_waist, 0)[1]
        for ch in chains:
            a0p = ch.particles[0]
            if a0p.kinematic and 0 <= a0p.bone < len(nodes):
                if bone_world_pos(a0p.bone, 0)[1] <= _wy + 0.05:
                    for p in ch.particles:
                        if not p.kinematic:
                            skirt_rbs.add(p.rb)
    init_pos = {}
    for ch in chains:
        for p in ch.particles:
            if (not p.kinematic) and (p.rb in skirt_rbs) and 0 <= p.bone < len(nodes):
                init_pos[p.rb] = bone_world_pos(p.bone, 0)

    # クロスソルバでベイク（縦チェーン＋横距離拘束）。髪は lateral=[] で従来同等。
    state = SpringState(chains, init_pos=init_pos)
    dt = 1.0 / fps
    warmup = 20   # 修正1のinit_posシードで十分。warmup増は髪の揺れを損なうため20維持
    a0 = anchor_fn(0)
    for _ in range(warmup):
        for rb, (wp, wr) in a0.items():
            state.set_anchor(rb, wp, wr)
        simulate_step_cloth(state, gravity_dir, dt, drag_force, stiffness_force,
                            lateral, gravity_power=gravity_power, iterations=6,
                            colliders=colliders_at(0), body_axis=body_axis_at(0),
                            radial_rbs=skirt_rbs)
    # 剛体変換の逆・点変換（translation焼き用）
    def _inv_rigid(M):
        Rt = [[M[0][0], M[1][0], M[2][0]],
              [M[0][1], M[1][1], M[2][1]],
              [M[0][2], M[1][2], M[2][2]]]
        t = (M[0][3], M[1][3], M[2][3])
        ti = tuple(-(Rt[i][0]*t[0] + Rt[i][1]*t[1] + Rt[i][2]*t[2]) for i in range(3))
        return [[Rt[0][0], Rt[0][1], Rt[0][2], ti[0]],
                [Rt[1][0], Rt[1][1], Rt[1][2], ti[1]],
                [Rt[2][0], Rt[2][1], Rt[2][2], ti[2]],
                [0.0, 0.0, 0.0, 1.0]]

    def _apply(M, p):
        return (M[0][0]*p[0] + M[0][1]*p[1] + M[0][2]*p[2] + M[0][3],
                M[1][0]*p[0] + M[1][1]*p[1] + M[1][2]*p[2] + M[1][3],
                M[2][0]*p[0] + M[2][1]*p[1] + M[2][2]*p[2] + M[2][3])

    keys = {}    # bone -> [(f, quat_local)]
    tkeys = {}   # bone -> [(f, (x,y,z) local translation)]  スカート等のみ
    for f in range(num_frames):
        a = anchor_fn(f)
        for rb, (wp, wr) in a.items():
            state.set_anchor(rb, wp, wr)
        seg = simulate_step_cloth(state, gravity_dir, dt, drag_force,
                                  stiffness_force, lateral,
                                  gravity_power=gravity_power, iterations=6,
                                  colliders=colliders_at(f), body_axis=body_axis_at(f),
                                  radial_rbs=skirt_rbs)
        loc = seg_rot_to_local(state, seg)
        for bone, q in loc.items():
            keys.setdefault(bone, []).append((f, q))
        # スカート(skirt_rbs)は位置も焼く: 回転のみだとボーン長が rest 固定で、
        # 衝突押し出し(3D)を表現できず貫入が残るため。世界行列を top-down に構築し、
        # 各スカートボーンの局所translationでパーティクル世界位置に正確に一致させる。
        for ch in chains:
            plist = ch.particles
            a0p = plist[0]
            if a0p.kinematic and a0p.rb in a:
                (apx, apy, apz), aq = a[a0p.rb]
                pw = trs_to_mat((apx, apy, apz), aq)
            else:
                pw = mat_ident()
            for k in range(1, len(plist)):
                c = plist[k]
                lq = loc.get(c.bone, (0.0, 0.0, 0.0, 1.0))
                if c.rb in skirt_rbs:
                    lt = _apply(_inv_rigid(pw), state.pos[c.rb])
                    tkeys.setdefault(c.bone, []).append((f, lt))
                    pw = mat_mul(pw, trs_to_mat(lt, lq))
                else:
                    rdir = state.rest_dir.get(c.rb, (0.0, 0.0, 0.0))
                    rl = c.rest_len
                    pw = mat_mul(pw, trs_to_mat((rdir[0]*rl, rdir[1]*rl, rdir[2]*rl), lq))
    out = {}
    for bone, ks in keys.items():
        ks.sort(key=lambda x: x[0])
        out[bone] = [q for _, q in ks]
    tout = {}
    for bone, ts in tkeys.items():
        ts.sort(key=lambda x: x[0])
        tout[bone] = [t for _, t in ts]
    return out, tout, len(excluded)


# ======================================================================
# クロス対応: BFSベースのチェーン抽出（リング構造を正しく分解）
# ======================================================================
def extract_chains_bfs(physics_gltf, bone_world_matrices, only_names=None):
    """アンカー(mode0剛体)からのBFSで親を決め、縦チェーン＋横距離拘束に分解。
    髪(単純チェーン)もスカート(リング)も統一的に扱える。

    戻り値: (chains, parts, excluded, lateral)
      lateral = [(rb_i, rb_j, rest_len), ...]  非ツリー辺（横リング）の距離拘束
    """
    rbs = physics_gltf["rigidBodies"]
    jts = physics_gltf["joints"]
    bwm = bone_world_matrices

    def target(rb):
        if only_names is None:
            return True
        return any(k in rb["name"] for k in only_names)

    def bpos(bi):
        m = bwm[bi]
        return (m[0][3], m[1][3], m[2][3]) if (0 <= bi < len(bwm) and m) else None

    # 無向隣接（joint経由）。辺ごとに joint を保持
    adj = {}   # rb_index -> list of (neighbor_rb, joint)
    for j in jts:
        a, b = j["rigidA"], j["rigidB"]
        adj.setdefault(a, []).append((b, j))
        adj.setdefault(b, []).append((a, j))

    # 対象 dynamic 剛体
    dyn = set(i for i, rb in enumerate(rbs) if rb["mode"] in (1, 2) and target(rb))
    # アンカー = dynでない剛体で、dynに隣接するもの（mode0の親）
    anchors = set()
    for i in dyn:
        for nb, j in adj.get(i, []):
            if nb not in dyn:
                anchors.add(nb)

    # Particle 生成
    parts = {}
    for i in dyn:
        p = Particle(rbs[i]["rb"] if False else i, rbs[i]["bone"], rbs[i]["mass"],
                     kinematic=False)
        p.rest_pos = bpos(rbs[i]["bone"])
        parts[i] = p
    for i in anchors:
        p = Particle(i, rbs[i]["bone"], rbs[i]["mass"], kinematic=True)
        p.rest_pos = bpos(rbs[i]["bone"])
        parts[i] = p

    # BFS（全アンカー同時開始）→ 親(ツリー辺)・深さ確定、非ツリー辺=横拘束
    from collections import deque
    depth = {i: 0 for i in anchors}
    tree_joint = {}     # child_rb -> joint（親との辺）
    dq = deque(anchors)
    visited = set(anchors)
    lateral = []
    seen_pairs = set()
    while dq:
        cur = dq.popleft()
        for nb, j in adj.get(cur, []):
            if nb not in parts:      # 対象外(別カテゴリのdyn等)は無視
                continue
            if nb not in visited:
                visited.add(nb)
                depth[nb] = depth[cur] + 1
                parts[nb].parent = cur
                tree_joint[nb] = j
                dq.append(nb)
            else:
                # 既訪問 = 非ツリー辺（横リング等）。距離拘束として1回だけ登録
                if nb in dyn or cur in dyn:
                    key = frozenset((cur, nb))
                    if key not in seen_pairs and cur != nb:
                        seen_pairs.add(key)
                        # ツリーの親子辺は除外（それは距離拘束で別途張る）
                        if parts[nb].parent != cur and parts[cur].parent != nb:
                            rp_a, rp_b = parts[cur].rest_pos, parts[nb].rest_pos
                            if rp_a and rp_b:
                                lateral.append((cur, nb, _len(_sub(rp_a, rp_b))))

    # rest_len / 角度制限 を tree_joint から埋める
    for c_rb, p in parts.items():
        if p.kinematic or p.parent == -1:
            continue
        j = tree_joint.get(c_rb)
        if j:
            p.ang_min = j["angularLimitMin"]
            p.ang_max = j["angularLimitMax"]
        pa = parts[p.parent]
        if p.rest_pos and pa.rest_pos:
            p.rest_len = _len(_sub(p.rest_pos, pa.rest_pos))

    # 到達不能(アンカー無)を除外
    excluded = set(i for i in dyn if parts[i].parent == -1)

    # チェーン化: 各 dynamic を chain_root(最上位dynamic)でグループ化
    def chain_root(i):
        seen = 0
        while parts[i].parent != -1 and not parts[parts[i].parent].kinematic:
            i = parts[i].parent; seen += 1
            if seen > 500:
                break
        return i

    groups = {}
    for i in dyn:
        if i in excluded:
            continue
        groups.setdefault(chain_root(i), []).append(i)

    chains = []
    for r, members in groups.items():
        members.sort(key=lambda i: depth.get(i, 0))
        ch = Chain()
        anchor_i = parts[members[0]].parent
        if anchor_i in parts and parts[anchor_i].kinematic:
            ch.particles.append(parts[anchor_i])
        for i in members:
            ch.particles.append(parts[i])
        chains.append(ch)
    chains.sort(key=lambda c: -len(c))
    return chains, parts, excluded, lateral


# ======================================================================
# クロス対応ソルバ: 積分 → 縦横拘束を反復 → 回転出力（髪も包含）
# ======================================================================
def simulate_step_cloth(state, gravity_dir, dt, drag_force, stiffness_force,
                        lateral, gravity_power=0.02, iterations=6,
                        colliders=None, body_axis=None, radial_rbs=None):
    """縦チェーン＋横距離拘束を反復して解くクロスソルバ。
    lateral: [(rb_i, rb_j, rest_len), ...]
    戻り値: seg_rot[rb]
    """
    pos, prev, part, rest_dir = state.pos, state.prev, state.part, state.rest_dir
    anchor_rot = getattr(state, "_anchor_rot", {})
    last_seg = getattr(state, "_last_seg", {})
    stiff = stiffness_force * dt
    grav = _scale(_norm(gravity_dir), gravity_power * dt) if gravity_power else (0.0, 0.0, 0.0)

    # --- 1. 積分（慣性＋重力＋rest方向へのstiffness nudge） ---
    for rb, p in part.items():
        if p.kinematic:
            continue
        pa = part.get(p.parent)
        q_par = (anchor_rot.get(p.parent, (0, 0, 0, 1)) if (pa and pa.kinematic)
                 else last_seg.get(p.parent, (0, 0, 0, 1)))
        tgt_dir = q_rotate_vec(q_par, rest_dir[rb]) if rb in rest_dir else (0, 0, 0)
        inertia = _scale(_sub(pos[rb], prev[rb]), 1.0 - drag_force)
        prev[rb] = pos[rb]
        nxt = _add(pos[rb], inertia)
        nxt = _add(nxt, grav)
        if tgt_dir != (0, 0, 0):
            nxt = _add(nxt, _scale(tgt_dir, stiff))
        pos[rb] = nxt

    # --- 2. 拘束を反復（縦length＋角度、横length、アンカー固定） ---
    for _ in range(iterations):
        # 縦: 各チェーン top-down（length + 角度クランプ）
        for ch in state.chains:
            plist = ch.particles
            q_par = anchor_rot.get(plist[0].rb, (0, 0, 0, 1)) if plist[0].kinematic \
                else (0, 0, 0, 1)
            for k in range(1, len(plist)):
                c = plist[k]; pa = plist[k-1]
                rdir = rest_dir[c.rb]
                tgt_dir = q_rotate_vec(q_par, rdir)
                if c.inv_mass > 0:
                    d = _sub(pos[c.rb], pos[pa.rb]); dl = _len(d)
                    if dl > 1e-9:
                        cur_dir = _scale(d, 1.0 / dl)
                        # 角度クランプ
                        if c.ang_max:
                            amax = max(abs(c.ang_max[0]), abs(c.ang_max[2]))
                            if amax > 1e-4:
                                cang = max(-1.0, min(1.0, _dot(tgt_dir, cur_dir)))
                                if cang < math.cos(amax):
                                    ax = _cross(tgt_dir, cur_dir)
                                    if _len(ax) > 1e-9:
                                        cur_dir = _rotate_about(tgt_dir, _norm(ax), amax)
                        pos[c.rb] = _add(pos[pa.rb], _scale(cur_dir, c.rest_len))
                q_par = q_mul(q_from_to(tgt_dir, _norm(_sub(pos[c.rb], pos[pa.rb]))), q_par)
        # 横: 距離拘束（両者dynamicなのでinv_mass比で配分）
        for a, b, rl in lateral:
            pa_, pb_ = part.get(a), part.get(b)
            if not pa_ or not pb_:
                continue
            d = _sub(pos[b], pos[a]); dist = _len(d)
            if dist < 1e-9:
                continue
            wa, wb = pa_.inv_mass, pb_.inv_mass
            ws = wa + wb
            if ws <= 0:
                continue
            corr = _scale(d, (dist - rl) / dist)
            pos[a] = _add(pos[a], _scale(corr, wa / ws))
            pos[b] = _sub(pos[b], _scale(corr, wb / ws))
        # コライダー衝突（脚カプセル等への push-out）
        if colliders:
            resolve_collisions(state, colliders, body_axis=body_axis,
                               radial_rbs=radial_rbs)
        # アンカーは常に固定位置へ（set_anchor で pos は既に固定済み）

    # 反復後にもう一度押し出し: 直前の length 拘束が侵入を復活させても、
    # 記録される最終位置は必ずコライダー外になるようにする（修正2）。
    if colliders:
        resolve_collisions(state, colliders, body_axis=body_axis,
                           radial_rbs=radial_rbs)

    # --- 3. 回転出力（top-down, 捻り保存）＋ last_seg キャッシュ ---
    seg_rot = {}
    for ch in state.chains:
        plist = ch.particles
        q_par = anchor_rot.get(plist[0].rb, (0, 0, 0, 1)) if plist[0].kinematic \
            else (0, 0, 0, 1)
        for k in range(1, len(plist)):
            c = plist[k]; pa = plist[k-1]
            cur_dir = _norm(_sub(pos[c.rb], pos[pa.rb]))
            aim = q_from_to(q_rotate_vec(q_par, rest_dir[c.rb]), cur_dir)
            seg_rot[c.rb] = q_mul(aim, q_par)
            q_par = seg_rot[c.rb]
    state._last_seg = seg_rot
    return seg_rot


# ======================================================================
# コライダー衝突（push-out）: スカート等が脚カプセルを突き抜けないように
# ======================================================================
def _closest_on_segment(p, a, b):
    ab = _sub(b, a)
    denom = _dot(ab, ab)
    if denom < 1e-12:
        return a
    t = _dot(_sub(p, a), ab) / denom
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    return _add(a, _scale(ab, t))

def resolve_collisions(state, colliders, body_axis=None, margin=0.0,
                       radial_rbs=None):
    """dynamic パーティクルを各コライダーの外へ押し出す（片方向）。

    body_axis: (center, up) 下半身(腰)の世界位置と縦軸。指定時、かつ対象が
      radial_rbs に含まれ、コライダーが腰より下(脚)のとき「体中心軸からの放射
      外向き」に押し出す。深い侵入でも内側=逆側へ貫通せず、内股のひだが体中心へ
      押し込まれない。それ以外は従来の最近傍表面方向（髪等の挙動を壊さない）。
    radial_rbs: 放射押し出しの対象 rb 集合（スカート等の下半身系のみ）。None=全て。
    colliders: [("capsule", p0, p1, radius), ("sphere", center, radius), ...]
    """
    if not colliders:
        return
    pos, part = state.pos, state.part
    bc = bup = None
    if body_axis is not None:
        bc, bup = body_axis

    def body_out(P, u_axis):
        """体中心軸からの水平放射外向き（u_axis=コライダー軸があれば直交化）。"""
        if bc is None:
            return None
        rel = _sub(P, bc)
        horiz = _sub(rel, _scale(bup, _dot(rel, bup)))  # 縦成分を除去
        if _len(horiz) < 1e-6:
            return None
        n = _norm(horiz)
        if u_axis is not None:
            n = _sub(n, _scale(u_axis, _dot(n, u_axis)))
            if _len(n) < 1e-6:
                return None
            n = _norm(n)
        return n

    for rb, p in part.items():
        if p.kinematic:
            continue
        P = pos[rb]
        for col in colliders:
            kind = col[0]
            if kind == "capsule":
                _, a, b, rad = col
                C = _closest_on_segment(P, a, b)
                u_axis = _norm(_sub(b, a))
            elif kind == "sphere":
                _, C, rad = col
                u_axis = None
            else:
                continue
            dvec = _sub(P, C)
            d = _len(dvec)
            R = rad + margin
            if d >= R:
                continue
            # 放射外向きは「腰より下のコライダー(脚)」かつ「下半身系パーティクル」限定。
            # 髪等・腰より上のコライダー(頭)は従来の表面法線に戻す。
            # 放射は「深い侵入(d < R*0.5=軸に近く逆側へ貫通しうる)」のみ。
            # 腰際の最上段スカートが腰コライダーと常時浅く重なる場合は最近傍を使い、
            # 放射の毎フレーム外向き累積による「腰への巻き込み・戻らない」を防ぐ。
            use_radial = ((bc is not None) and (C[1] < bc[1])
                          and (radial_rbs is None or rb in radial_rbs)
                          and (d < R * 0.5))
            n = body_out(P, u_axis) if use_radial else None
            if n is None:
                if d > 1e-9:
                    P = _add(C, _scale(dvec, R / d))
                else:
                    P = _add(C, (R, 0.0, 0.0))
                continue
            # 放射方向 n に沿って軸距離が R になる t>=0 を解く（逆側貫通しない）
            perp = _sub(P, C)
            pn = _dot(perp, n)
            pp = _dot(perp, perp)
            disc = pn * pn - (pp - R * R)
            if disc < 0.0:
                if d > 1e-9:
                    P = _add(C, _scale(dvec, R / d))
                continue
            t = -pn + math.sqrt(disc)
            if t < 0.0:
                t = 0.0
            P = _add(P, _scale(n, t))
        pos[rb] = P
