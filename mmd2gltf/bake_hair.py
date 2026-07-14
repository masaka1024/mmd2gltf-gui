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

# このファイルが実際にどのビルドかをログで確認するためのバージョン識別子。
# ベイク実行のたびに [physics] ログの先頭に出力する。内容を変更した際は必ず
# ここも更新し、環境側のファイルが古い/キャッシュされている疑いを
# ログだけで切り分けられるようにする。
BAKE_HAIR_VERSION = "2026-07-14h (hem_extra_margin now smoothly interpolates root->hem)"

def _sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def _len(a): return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

class Particle:
    __slots__ = ("rb", "bone", "mass", "inv_mass", "parent", "rest_len",
                 "rest_dir_local", "ang_min", "ang_max", "rest_pos", "kinematic",
                 "group", "no_collision_mask")
    def __init__(self, rb, bone, mass, kinematic, group=0, no_collision_mask=0):
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
        self.group = group                      # MMDの非衝突グループ番号(0-15)
        self.no_collision_mask = no_collision_mask  # このグループとは衝突しない、というビットマスク

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
                    seed = p.rest_pos if p.rest_pos is not None else (0.0, 0.0, 0.0)
                    self.pos[p.rb] = seed
                    self.prev[p.rb] = seed

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
                    if seed is None:
                        seed = p.rest_pos
                    if seed is None:
                        seed = (0.0, 0.0, 0.0)  # rest_pos 欠落時の保険
                    self.pos[p.rb] = seed
                    self.prev[p.rb] = seed   # prev も同位置 = 初速ゼロ
            plist = ch.particles
            for k in range(1, len(plist)):
                c = plist[k]; pa = plist[k-1]
                if c.rest_pos and pa.rest_pos:
                    self.rest_dir[c.rb] = _norm(_sub(c.rest_pos, pa.rest_pos))
                else:
                    self.rest_dir[c.rb] = (0.0, -1.0, 0.0)  # 安全側の既定方向

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

def _q_angle(a, b):
    """クォータニオン a, b の角度差(度)。"""
    d = max(-1.0, min(1.0, abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3])))
    return math.degrees(2.0 * math.acos(d))


def q_slerp_toward(cur, prev, max_deg):
    """cur を、prev から max_deg 以内(実回転角)に収まるよう球面補間で引き戻す。
    差が max_deg 以下ならそのまま cur を返す（無干渉）。
    数値的異常(NaN等)を伴う特異点由来の跳ねを、前フレーム基準に安全側で抑える。

    注意: クォータニオンの内積から出る角 theta=acos(dot) は「半角」
    （実回転角 = 2*theta）。半角ベースで比較・補間しないと、実際に許容した
    角度の2倍だけ動いてしまうので、常に半角(max_deg/2)で扱う。
    """
    if prev is None:
        return cur
    a, b = prev, cur
    dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
    if dot < 0.0:
        b = (-b[0], -b[1], -b[2], -b[3]); dot = -dot
    dot = max(-1.0, min(1.0, dot))
    theta = math.acos(dot)               # 半角（実回転角の半分）
    max_half_rad = math.radians(max_deg) * 0.5
    if theta <= max_half_rad or theta < 1e-9:
        return cur
    t = max_half_rad / theta
    sin_theta = math.sin(theta)
    wa = math.sin((1.0 - t) * theta) / sin_theta
    wb = math.sin(t * theta) / sin_theta
    out = (a[0]*wa + b[0]*wb, a[1]*wa + b[1]*wb,
           a[2]*wa + b[2]*wb, a[3]*wa + b[3]*wb)
    n = math.sqrt(sum(c*c for c in out)) or 1.0
    return (out[0]/n, out[1]/n, out[2]/n, out[3]/n)



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
                        collision_margin=0.0,
                        drag_force=0.85, stiffness_force=1.5, gravity_power=0.02,
                        gravity_dir=(0.0, -1.0, 0.0), fps=30.0,
                        only_names=("髪",), force_no_collision_names=None,
                        allowed_collider_names=None, hem_extra_margin=0.0):
    """体アニメ baked を駆動源に、髪ボーンのローカル回転キーを生成する。

    gltf_json    : 組み上がった g.j（nodes 必須）
    baked        : {node_index: {"r":[MMD quat...], "t":[MMD vec3...]}}（bake()の戻り）
    num_frames   : len(times)
    physics_gltf : extras.mmd.physicsGltf（build_physics_gltf の出力）
    scale        : unitScale
    force_no_collision_names : 剛体名の集合(set/list)。指定すると、その名前に
        一致する剛体を全グループ非衝突として扱う(PMXのgroup/noCollisionMask
        設定を上書き)。デフォルトNoneなら何もしない(既存挙動と完全に同じ)。
        PMX側のnoCollisionMaskが正しく保存されていない、あるいはMMD側が
        正しく反映していない可能性がある剛体を、変換時に個別に除外するための
        逃げ道。両側(揺れ物の粒子側・静的コライダー側)どちらの名前を指定しても
        機能する(例: {"下半身02"} や、髪/スカートのグループをまとめて除外する
        名前リストなど)。
    allowed_collider_names : コライダー剛体名の集合(set/list)。指定すると、
        揺れ物の衝突判定を「デナイリスト方式」(MMD/PMX標準: 全コライダーと
        衝突する。noCollisionMask/force_no_collision_namesで個別に除外する)
        から「アローリスト方式」(VRM SpringBone/VRChat PhysBones方式: この
        リストに列挙したコライダーだけを見る。それ以外は最初から存在しない
        ものとして扱う)へ切り替える。デフォルトNoneならデナイリスト方式
        (既存挙動と完全に同じ)。指定した場合、PMXのgroup/noCollisionMaskや
        force_no_collision_namesは一切参照されない(アローリストが唯一の
        判定基準になる)。モデル制作者が非衝突グループの設定を誤った/MMD側が
        正しく反映していない可能性がある場合に、脚・腰など本当に必要な
        コライダーだけを明示して安全に運用するための代替モード。
    hem_extra_margin : 各スカートチェーンの根元(腰側)から裾にかけて、0→この値まで
        滑らかに線形補間しながら collision_margin へ上乗せする追加クリアランス
        (根元=0、裾=hem_extra_margin全量)。デフォルト0.0で無効(既存挙動と
        完全に同じ)。太もも等へのメッシュ面の見た目上の食い込みは裾で起きやすい
        一方、collision_margin自体を全体で上げると腰に近いセグメントが押し
        出されて「傘化」に近づいてしまう(実測: margin 0.08でほぼ元の傘化
        相当まで戻る)ため、根元は完全に不変のまま裾へ向けて安全にクリアランスを
        稼ぐための逃げ道。裾の1パーティクルだけをON/OFFする段差ではなく、
        チェーンの深さに比例して連続的に変化するので不自然な継ぎ目が出ない。
    戻り値       : { node_index: [ (x,y,z,w), ... num_frames ] }（glTF局所回転）
    """
    nodes = gltf_json["nodes"]
    print("  [physics] bake_hair.py version: %s" % BAKE_HAIR_VERSION)
    bwm = compute_bone_world_matrices(gltf_json)
    chains, parts, excluded, lateral = extract_chains_bfs(
        physics_gltf, bwm, only_names=only_names)
    rbs_names_all = physics_gltf["rigidBodies"]
    if force_no_collision_names:
        _force_set = set(force_no_collision_names)
        _forced = 0
        for p in parts.values():
            if rbs_names_all[p.rb]["name"] in _force_set:
                p.no_collision_mask = 0xFFFF
                _forced += 1
        if _forced:
            print("  [physics] force_no_collision_names: %d particle(s) forced "
                  "to non-colliding (%s)" % (_forced, sorted(_force_set)))

    allowed_collider_rbi = None
    if allowed_collider_names is not None:
        _allow_set = set(allowed_collider_names)
        allowed_collider_rbi = set(i for i, rb in enumerate(rbs_names_all)
                                   if rb["name"] in _allow_set)
        print("  [physics] allowed_collider_names: allowlist mode ON, "
              "%d/%d requested name(s) matched a rigid body (%s)"
              % (len(allowed_collider_rbi), len(_allow_set), sorted(_allow_set)))

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
    colliders_def = []   # (shape, bone_path, size, local_pos, local_rot, group, mask, rb_index)
    for _rbi, rb in enumerate(rbs_pg):
        if rb.get("mode") == 0 and rb.get("shape") in (0, 2):  # 0=球 2=カプセル
            bi = rb.get("bone", -1)
            if not (0 <= bi < len(nodes)):
                continue
            path = []
            bb = bi
            while bb != -1:
                path.append(bb); bb = parent[bb]
            path.reverse()
            _mask = rb.get("noCollisionMask", 0)
            if force_no_collision_names and rb["name"] in force_no_collision_names:
                _mask = 0xFFFF
            colliders_def.append((rb["shape"], path, rb["size"],
                                  rb["position"], rb["rotation"],
                                  rb.get("group", 0), _mask, _rbi))

    def colliders_at(f):
        cols = []
        for shape, path, size, lpos, lrot, group, mask, rbi in colliders_def:
            W = world_at(path, f)
            M = mat_mul(W, trs_to_mat(lpos, lrot))
            c = (M[0][3], M[1][3], M[2][3])
            if shape == 0:               # 球
                cols.append(("sphere", c, size[0], group, mask, rbi))
            else:                        # カプセル（長軸=ローカルY）
                _, wq = mat_to_trs(M)
                ay = q_rotate_vec(wq, (0.0, 1.0, 0.0))
                half = size[1] * 0.5
                p0 = (c[0] - ay[0]*half, c[1] - ay[1]*half, c[2] - ay[2]*half)
                p1 = (c[0] + ay[0]*half, c[1] + ay[1]*half, c[2] + ay[2]*half)
                cols.append(("capsule", p0, p1, size[0], group, mask, rbi))
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

    # 各スカートチェーンの「裾からの深さ比率」。根元=0.0、裾=1.0、間は線形補間。
    # hem_extra_margin は margin へ hem_weight[rb] 倍だけ上乗せする(裾ほど強く、
    # 根元では0=無効)。ON/OFFの段差ではなく、根元から裾へ滑らかに繋げるため。
    hem_weight = {}
    for ch in chains:
        dyn = [p for p in ch.particles if p.rb in skirt_rbs and not p.kinematic]
        k = len(dyn)
        if k == 0:
            continue
        for i, p in enumerate(dyn):
            hem_weight[p.rb] = 1.0 if k == 1 else i / (k - 1)

    # 横拘束(lateral)はスカートのリング(2次元クロス)専用。髪・ネクタイ・胸などの
    # チェーン型揺れ物は、モデルにストランド間ジョイントがあると非ツリー辺として
    # lateral 化され、ハード距離拘束で2次元網に固まり中間ボーンが偽平衡に固着する
    # (実測: IAの髪FLB2が頭姿勢に無関係に67°固着。lateral除去で静止1.4°/動的に解放)。
    # lateral は両端がスカート剛体のものだけ残す(髪等のチェーンは lateral=[] が本来の設計)。
    _lat_before = len(lateral)
    lateral = [(a, b, rl) for (a, b, rl) in lateral
               if a in skirt_rbs and b in skirt_rbs]
    if _lat_before != len(lateral):
        print("  [physics] dropped %d non-skirt lateral constraint(s) "
              "(hair/necktie/etc. are chains, not cloth)"
              % (_lat_before - len(lateral)))

    init_pos = {}
    for ch in chains:
        for p in ch.particles:
            if (not p.kinematic) and (p.rb in skirt_rbs) and 0 <= p.bone < len(nodes):
                init_pos[p.rb] = bone_world_pos(p.bone, 0)

    # チェーンのアンカー自身がコライダー(mode0球/カプセル)でもある場合の除外集合。
    # 例: 裾を脚に密着させる設計だと、アンカー(右足/左足)剛体がそのまま脚コライダー
    # でもあり、チェーンの拘束(アンカーへ引き寄せる)と衝突押し出しが毎フレーム
    # 綱引きして震動する。そのチェーンの全パーティクルから、アンカーと同じ
    # コライダーだけを除外する（他のチェーン・他の揺れ物には影響しない）。
    _collider_rbs = set(c[-1] for c in colliders_def)
    exclude_rb = {}
    for ch in chains:
        a0p = ch.particles[0]
        if a0p.kinematic and a0p.rb in _collider_rbs:
            excl = {a0p.rb}
            for p in ch.particles:
                if not p.kinematic:
                    exclude_rb.setdefault(p.rb, set()).update(excl)

    # スカート(translation焼き対象)ボーンの回転出力を恒等(親と同じ向き)に固定する。
    # 理由: これらのボーンは position(translation) で世界位置を厳密一致させている
    # ため、回転(向き)の値そのものはメッシュのツイスト表現以外に意味を持たない。
    # 一方で回転計算(q_from_to による rest 基準の目標方向)は、乱れたツリー構造
    # (リング横断等)を持つ揺れ物では目標方向と実方向がほぼ正反対になる瞬間があり、
    # 特異点で軸の選び方が不連続になって1フレームで大きく跳ねることがある(実測109°)。
    # 位置はtranslationで担保済みなので、この跳ねを恒等固定で完全に無効化する。
    skirt_bone_set = set(p.bone for ch in chains for p in ch.particles if p.rb in skirt_rbs)

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
                            radial_rbs=skirt_rbs, margin=collision_margin,
                            exclude_rb=exclude_rb, skip_angle_clamp_rbs=skirt_rbs,
                            allowed_collider_rbi=allowed_collider_rbi,
                            hem_weight=hem_weight, hem_extra_margin=hem_extra_margin)

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
    _ndyn = sum(1 for ch in chains for pp in ch.particles if not pp.kinematic)
    print("  [physics] baking %d cloth bones over %d frames..."
          % (_ndyn, num_frames))
    for f in range(num_frames):
        if f % 500 == 0 and f > 0:
            print("  [physics]   frame %d/%d" % (f, num_frames))
        a = anchor_fn(f)
        for rb, (wp, wr) in a.items():
            state.set_anchor(rb, wp, wr)
        seg = simulate_step_cloth(state, gravity_dir, dt, drag_force,
                                  stiffness_force, lateral,
                                  gravity_power=gravity_power, iterations=6,
                                  colliders=colliders_at(f), body_axis=body_axis_at(f),
                                  radial_rbs=skirt_rbs, margin=collision_margin,
                                  exclude_rb=exclude_rb, skip_angle_clamp_rbs=skirt_rbs,
                                  allowed_collider_rbi=allowed_collider_rbi,
                                  hem_weight=hem_weight, hem_extra_margin=hem_extra_margin)
        loc = seg_rot_to_local(state, seg)
        for bone in skirt_bone_set:
            if bone in loc:
                loc[bone] = (0.0, 0.0, 0.0, 1.0)
        for bone, q in loc.items():
            keys.setdefault(bone, []).append((f, q))
        # スカート(skirt_rbs)は位置も焼く: 回転のみだとボーン長が rest 固定で、
        # 衝突押し出し(3D)を表現できず貫入が残るため。世界行列を top-down に構築し、
        # 各スカートボーンの局所translationでパーティクル世界位置に正確に一致させる。
        # 複数剛体が同一ボーンを共有する場合、そのボーンには1フレーム1回だけ記録する
        # （でないと出力アクセサのカウントが frame数×共有数 に膨れ、glTF検証エラー）。
        t_written = set()
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
                    if c.bone not in t_written:
                        tkeys.setdefault(c.bone, []).append((f, lt))
                        t_written.add(c.bone)
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

    # 対象 dynamic 剛体。rest_pos が取れない(bone=-1 等でNone)剛体は
    # 位置も回転も焼けないため最初から除外する（None混入によるクラッシュ防止）。
    dyn = set(i for i, rb in enumerate(rbs)
              if rb["mode"] in (1, 2) and target(rb) and bpos(rb["bone"]) is not None)
    # アンカー = dynでない剛体で、dynに隣接するもの（mode0の親）。
    # アンカーも rest_pos が None のものは使えないので除外。
    anchors = set()
    for i in dyn:
        for nb, j in adj.get(i, []):
            if nb not in dyn and bpos(rbs[nb]["bone"]) is not None:
                anchors.add(nb)

    # Particle 生成
    parts = {}
    for i in dyn:
        p = Particle(rbs[i]["rb"] if False else i, rbs[i]["bone"], rbs[i]["mass"],
                     kinematic=False,
                     group=rbs[i].get("group", 0),
                     no_collision_mask=rbs[i].get("noCollisionMask", 0))
        p.rest_pos = bpos(rbs[i]["bone"])
        parts[i] = p
    for i in anchors:
        p = Particle(i, rbs[i]["bone"], rbs[i]["mass"], kinematic=True,
                     group=rbs[i].get("group", 0),
                     no_collision_mask=rbs[i].get("noCollisionMask", 0))
        p.rest_pos = bpos(rbs[i]["bone"])
        parts[i] = p

    # Dijkstra（全アンカー同時開始、辺重み=rest距離）→ 親(ツリー辺)確定、非ツリー辺=横拘束。
    # 単純なホップ数BFSだと、リング接続(同じ段の輪)を辿った方がアンカーから
    # 少ないホップで届くことがあり、その場合はリングを横断する不自然な親子関係
    # (例: 上段の輪を1周してから下段に接続)になりやすい。これは実際の物理挙動として
    # 不安定（アンカーの動きが輪全体に梃子のように伝わり、回転出力が特異点近くで
    # 跳ねる)なので、ホップ数ではなく実際の3D距離を最短路の重みにする。
    import heapq
    depth = {i: 0.0 for i in anchors}
    tree_joint = {}     # child_rb -> joint（親との辺）
    visited = set(anchors)
    lateral = []
    seen_pairs = set()
    heap = [(0.0, i) for i in anchors]
    heapq.heapify(heap)
    while heap:
        dcur, cur = heapq.heappop(heap)
        if dcur > depth.get(cur, dcur):
            continue   # 古いエントリ（既により短い経路で確定済み）
        for nb, j in adj.get(cur, []):
            if nb not in parts:      # 対象外(別カテゴリのdyn等)は無視
                continue
            rp_c, rp_n = parts[cur].rest_pos, parts[nb].rest_pos
            w = _len(_sub(rp_c, rp_n)) if (rp_c and rp_n) else 1.0
            nd = dcur + w
            if nb not in visited:
                visited.add(nb)
                depth[nb] = nd
                parts[nb].parent = cur
                tree_joint[nb] = j
                heapq.heappush(heap, (nd, nb))
            elif nd < depth.get(nb, float("inf")) - 1e-9:
                # より短い経路が後から見つかった（優先度キューの構造上あり得る）。
                # 親を更新し、旧親子辺は横拘束として残す。
                old_parent = parts[nb].parent
                if old_parent != -1 and old_parent != cur:
                    key = frozenset((old_parent, nb))
                    if key not in seen_pairs:
                        seen_pairs.add(key)
                        rp_a, rp_b = parts[old_parent].rest_pos, parts[nb].rest_pos
                        if rp_a and rp_b:
                            lateral.append((old_parent, nb, _len(_sub(rp_a, rp_b))))
                depth[nb] = nd
                parts[nb].parent = cur
                tree_joint[nb] = j
                heapq.heappush(heap, (nd, nb))
            else:
                # 既訪問・より短くない = 非ツリー辺（横リング等）。距離拘束として1回だけ登録
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

    # 警告用: 複数アンカー(腰＋脚など、輪を複数箇所で固定する構造)の検出。
    # ツリー辺(parent)＋横拘束(lateral)を辿った連結成分内に、異なるアンカー
    # (kinematic剛体)が複数あれば、裾を脚に密着させる等の「巻きつき」設計。
    # このソルバーは単一アンカー(腰のみ)のチェーンを主に想定しており、
    # 複数アンカー構造では衝突・振動が不安定になりやすい（既知の制限）。
    def true_anchor(i):
        seen = 0
        cur = i
        while parts[cur].parent != -1:
            cur = parts[cur].parent; seen += 1
            if seen > 1000:
                break
        return cur

    uf = {}
    def uf_find(x):
        while uf.get(x, x) != x:
            uf[x] = uf.get(uf[x], uf[x])
            x = uf[x]
        return x
    def uf_union(a, b):
        ra, rb = uf_find(a), uf_find(b)
        if ra != rb:
            uf[ra] = rb

    all_ids = set(dyn) | anchors
    for i in all_ids:
        uf.setdefault(i, i)
    for i in dyn:
        if i not in excluded:
            uf_union(i, true_anchor(i))
    for a, b, _rl in lateral:
        uf_union(a, b)

    comp_anchors = {}
    for i in dyn:
        if i in excluded:
            continue
        root = uf_find(i)
        comp_anchors.setdefault(root, set()).add(true_anchor(i))

    multi_anchor_groups = [(root, anc) for root, anc in comp_anchors.items() if len(anc) > 1]
    if multi_anchor_groups:
        seen_names = set()
        for root, anc in multi_anchor_groups:
            for a in anc:
                bi = rbs[a].get("bone", -1)
                if 0 <= bi < len(bone_world_matrices):
                    pass
                seen_names.add(rbs[a]["name"])
        names_list = sorted(seen_names)
        shown = names_list[:8]
        names_str = ", ".join(shown)
        if len(names_list) > 8:
            names_str += " 他%d件" % (len(names_list) - 8)
        print("  [physics] 警告: 複数アンカー構造を検出 (%d グループ, アンカー候補: %s)。"
              % (len(multi_anchor_groups), names_str))
        print("  [physics]   例: 裾を脚に密着させる等の「巻きつき」設計。腰のみアンカーの"
              "通常のスカート/髪と異なり、この構造は物理挙動が不安定(振動)になる場合が"
              "あります。目視で確認してください。")

    return chains, parts, excluded, lateral


# ======================================================================
# クロス対応ソルバ: 積分 → 縦横拘束を反復 → 回転出力（髪も包含）
# ======================================================================
def simulate_step_cloth(state, gravity_dir, dt, drag_force, stiffness_force,
                        lateral, gravity_power=0.02, iterations=6,
                        colliders=None, body_axis=None, radial_rbs=None,
                        margin=0.0, exclude_rb=None, skip_angle_clamp_rbs=None,
                        allowed_collider_rbi=None, hem_weight=None, hem_extra_margin=0.0):
    """縦チェーン＋横距離拘束を反復して解くクロスソルバ。
    lateral: [(rb_i, rb_j, rest_len), ...]
    戻り値: seg_rot[rb]
    """
    pos, prev, part, rest_dir = state.pos, state.prev, state.part, state.rest_dir
    anchor_rot = getattr(state, "_anchor_rot", {})
    last_seg = getattr(state, "_last_seg", {})
    stiff = stiffness_force * dt
    # 安全弁: このフレーム開始時点の位置を記録（最大変位クランプ用）
    frame_start_pos = dict(pos)
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
                        # 角度クランプ: スカート等(translation焼き対象)は対象外。
                        # 理由: 角度クランプは tgt_dir(=rest_dir を q_par で回転した
                        # もの)へ向けて実位置を強制する。複数アンカー・リング構造を
                        # 持つ揺れ物では q_par の元になる rest_dir が乱れたツリーの
                        # 影響で不安定になりやすく、その不安定な tgt_dir へ位置を
                        # 強制クランプすることで実際の位置が1フレームで大きく跳ねる
                        # (実測: 世界位置ジャンプ最大0.24)。この種のパーティクルは
                        # ring(横)拘束と衝突・重力だけで十分に形状が保たれるため、
                        # 角度クランプ無しでも破綻しない。
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
                               radial_rbs=radial_rbs, margin=margin,
                               exclude_rb=exclude_rb,
                               allowed_collider_rbi=allowed_collider_rbi,
                               hem_weight=hem_weight, hem_extra_margin=hem_extra_margin)
        # アンカーは常に固定位置へ（set_anchor で pos は既に固定済み）

    # 反復後にもう一度押し出し: 直前の length 拘束が侵入を復活させても、
    # 記録される最終位置は必ずコライダー外になるようにする（修正2）。
    if colliders:
        resolve_collisions(state, colliders, body_axis=body_axis,
                           radial_rbs=radial_rbs, margin=margin,
                           exclude_rb=exclude_rb,
                           allowed_collider_rbi=allowed_collider_rbi,
                           hem_weight=hem_weight, hem_extra_margin=hem_extra_margin)

        # 修正3: 上の念押し押し出しの後、長さ拘束だけを再適用してrest_lenへ戻す。
        # 押し出しが決めた方向(=衝突を避けた向き)はそのまま維持し、長さだけを
        # rest_len にスケールし直す(同じ方向への縮小なので新たな貫入は生まない)。
        # 実測: これが無いと、念押しの押し出しで伸びた分を戻す機会が二度と無く、
        # 伸びたまま確定する(実測: 極端な姿勢でセグメントが最大2.1倍まで伸びを確認、
        # 36セグメント中22本が5%超の伸び)。
        # 対象は radial_rbs(スカート等の下半身系)の子パーティクルのみに限定する。
        # 全チェーンに適用すると、衝突と無関係な胸物理・髪等のwarmup挙動まで
        # わずかに変わり、減衰の弱いチェーンで長時間シミュレーション後に大きく
        # 発散する副作用が実測で確認された(左胸先の回転が最大52.8°変化)。
        # radial_rbs限定なら、この副作用は原理上生じない(対象外チェーンの
        # posを一切変更しないため)。
        if radial_rbs:
            for ch in state.chains:
                plist = ch.particles
                for k in range(1, len(plist)):
                    c = plist[k]; pa = plist[k-1]
                    if c.rb not in radial_rbs:
                        continue
                    if c.inv_mass <= 0:
                        continue
                    d = _sub(pos[c.rb], pos[pa.rb]); dl = _len(d)
                    if dl > 1e-9:
                        pos[c.rb] = _add(pos[pa.rb], _scale(d, c.rest_len / dl))

    # 安全弁: 1フレームあたりの最大変位クランプ。乱れたツリー構造(複数アンカー・
    # リング横断)を持つ揺れ物は、rest_dir/角度クランプの相互作用で稀に位置が
    # 1フレームで大きく跳ねることがある(実測 最大0.5 unit)。物理的に妥当な動きは
    # rest_len の数倍を1フレームで超えることはまずないため、それを超える変位は
    # 方向を保ったまま長さだけ制限する（見た目の瞬間移動＝暴れを防ぐ安全網）。
    for rb, p in part.items():
        if p.kinematic:
            continue
        start = frame_start_pos.get(rb)
        if start is None:
            continue
        cap = max(p.rest_len, 0.01) * 3.0
        delta = _sub(pos[rb], start)
        dl = _len(delta)
        if dl > cap:
            pos[rb] = _add(start, _scale(delta, cap / dl))

    # --- 3. 回転出力（top-down, 捻り保存）＋ last_seg キャッシュ ---
    # 安全クランプ: rest_dir が乱れたツリー構造(リング横断など)の場合、目標方向と
    # 実方向がほぼ正反対に近づく瞬間に q_from_to が特異点(対蹠点)に入り、軸の
    # 選び方が不連続になって1フレームで大きく回転が跳ねることがある(実測109°等)。
    # 前フレームの回転から1フレームあたり最大 MAX_ROT_DEG だけ許容し、それを
    # 超える成分は球面補間で引き戻す。q_par(下流への伝播元)もクランプ後の値を
    # 使うため、跳ねが子孫パーティクルへ伝播するのも同時に防げる。
    MAX_ROT_DEG = 60.0
    seg_rot = {}
    for ch in state.chains:
        plist = ch.particles
        q_par = anchor_rot.get(plist[0].rb, (0, 0, 0, 1)) if plist[0].kinematic \
            else (0, 0, 0, 1)
        for k in range(1, len(plist)):
            c = plist[k]; pa = plist[k-1]
            cur_dir = _norm(_sub(pos[c.rb], pos[pa.rb]))
            aim = q_from_to(q_rotate_vec(q_par, rest_dir[c.rb]), cur_dir)
            raw = q_mul(aim, q_par)
            prev = last_seg.get(c.rb)
            clamped = q_slerp_toward(raw, prev, MAX_ROT_DEG)
            seg_rot[c.rb] = clamped
            q_par = clamped
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
                       radial_rbs=None, exclude_rb=None, allowed_collider_rbi=None,
                       hem_weight=None, hem_extra_margin=0.0):
    """dynamic パーティクルを各コライダーの外へ押し出す（片方向）。

    body_axis: (center, up) 下半身(腰)の世界位置と縦軸。指定時、かつ対象が
      radial_rbs に含まれ、コライダーが腰より下(脚)のとき「体中心軸からの放射
      外向き」に押し出す。深い侵入でも内側=逆側へ貫通せず、内股のひだが体中心へ
      押し込まれない。それ以外は従来の最近傍表面方向（髪等の挙動を壊さない）。
    radial_rbs: 放射押し出しの対象 rb 集合（スカート等の下半身系のみ）。None=全て。
    exclude_rb: {particle_rb: {collider_rb_index, ...}} 。パーティクルが所属する
      チェーンのアンカー自身がコライダーでもある場合（脚に密着させる裾など）、
      その組だけ衝突判定から除外する。アンカーへ引き寄せる拘束と押し出しが
      毎フレーム綱引きして震動するのを防ぐ。None なら除外なし。
    allowed_collider_rbi: rb_index の集合。指定すると「アローリスト方式」に
      切り替わり、このリストに無いコライダーは(group/noCollisionMaskの設定に
      関わらず)最初から存在しないものとして扱う。None なら従来の
      「デナイリスト方式」(group/noCollisionMaskで個別に除外)のまま。
    hem_weight: {rb_index: 0.0〜1.0} の辞書。各スカートチェーンの裾からの
      深さ比率(根元=0.0、裾=1.0、間は線形補間)。指定時、margin へ
      hem_extra_margin * hem_weight[rb] だけ上乗せする。太もも等への
      メッシュ面の見た目上の食い込みは裾で起きやすい一方、marginを全体で
      上げると腰に近いセグメントが押し出されて傘化に近づくため、根元から
      裾へ滑らかに繋げつつ安全にクリアランスを稼ぐ。
    colliders: [("capsule", p0, p1, radius, group, no_collision_mask, rb_index),
                ("sphere", center, radius, group, no_collision_mask, rb_index), ...]
    """
    if not colliders:
        return
    pos, part = state.pos, state.part
    bc = bup = None
    if body_axis is not None:
        bc, bup = body_axis

    # ブロードフェーズ用: 各コライダーの中心と「カル半径^2」を前計算。
    # パーティクルが中心からカル半径より遠ければ詳細判定(closest_on_segment)を省く。
    # 大規模モデル(多数の揺れ物×多数コライダー)で致命的に効く。
    # allowed_collider_rbi指定時は、ここでアローリスト外のコライダーを
    # 丸ごと弾く(以降の一切の判定から除外)。
    _bp = []   # (col, cx, cy, cz, cull_sq, rbi)
    _cull_margin = margin + max(hem_extra_margin, 0.0)  # 裾の上乗せ込みで保守的に見積もる
    for col in colliders:
        rbi = col[-1]
        if allowed_collider_rbi is not None and rbi not in allowed_collider_rbi:
            continue
        if col[0] == "capsule":
            _, a, b, rad, _grp, _msk, _rbi = col
            cx = (a[0] + b[0]) * 0.5; cy = (a[1] + b[1]) * 0.5; cz = (a[2] + b[2]) * 0.5
            half = 0.5 * math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2 + (b[2]-a[2])**2)
            cull = half + rad + _cull_margin
        elif col[0] == "sphere":
            _, c, rad, _grp, _msk, _rbi = col
            cx, cy, cz = c
            cull = rad + _cull_margin
        else:
            continue
        _bp.append((col, cx, cy, cz, cull * cull, rbi))

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
        px, py, pz = P
        excl = exclude_rb.get(rb) if exclude_rb else None
        _eff_margin = margin + hem_extra_margin * (hem_weight.get(rb, 0.0) if hem_weight else 0.0)
        for col, ccx, ccy, ccz, cull_sq, rbi in _bp:
            if excl is not None and rbi in excl:
                # このパーティクルのチェーンが直接ぶら下がっているコライダー自身。
                # 拘束が引き寄せ・衝突が押し出す綱引きで震動するため除外。
                continue
            # ブロードフェーズ: 中心から遠ければ詳細判定を省く
            dxc = px - ccx; dyc = py - ccy; dzc = pz - ccz
            if dxc*dxc + dyc*dyc + dzc*dzc > cull_sq:
                continue
            kind = col[0]
            if kind == "capsule":
                _, a, b, rad, cgroup, cmask, _rbi = col
                C = _closest_on_segment(P, a, b)
                u_axis = _norm(_sub(b, a))
            elif kind == "sphere":
                _, C, rad, cgroup, cmask, _rbi = col
                u_axis = None
            else:
                continue
            # MMDの非衝突グループ判定: PMXの剛体はgroup(0-15)とnoCollisionMask
            # (自分がどのgroupと衝突しないかのビットマスク)を持つ。モデル制作者が
            # 明示的に「このコライダーとこの揺れ物は衝突させない」と設定している
            # 場合があり(実測: あるモデルの下半身の球コライダーはスカートの
            # groupを、腕/ひじのカプセルは全groupを非衝突に設定していた)、これを
            # 無視すると本来当たらないはずのコライダーにスカートが押し当てられて
            # 伸びたり戻らなくなったりする。相手側のmaskが自分のgroupを、または
            # 自分のmaskが相手のgroupを除外していれば、双方向にスキップする。
            # アローリスト方式(allowed_collider_rbi指定)の場合は、この判定自体を
            # 行わない。アローリストに入っているコライダーは無条件に有効な相手と
            # みなす(group/noCollisionMaskの設定がどうであれ関係ない)。
            if allowed_collider_rbi is None:
                if (cmask & (1 << p.group)) or (p.no_collision_mask & (1 << cgroup)):
                    continue
            dvec = _sub(P, C)
            d = _len(dvec)
            R = rad + _eff_margin
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