# -*- coding: utf-8 -*-
"""剛体ベイク（PBD）。physicsGltf からチェーンを抽出し、体モーション下で
髪の揺れをシミュレートして髪ボーンの回転キーを生成する。

このファイルは段階実装中。まずは chain 抽出まで。
依存: 標準ライブラリのみ（physics.py のベクトル演算を流用）。
"""
import math
# 本体(パッケージ)では from .physics import に変更すること
from .physics import (q_mul, q_rotate_vec, q_conj, q_normalize,
                     compute_bone_world_matrices, trs_to_mat, mat_mul,
                     mat_ident, mat_to_trs, euler_to_quat)

# このファイルが実際にどのビルドかをログで確認するためのバージョン識別子。
# ベイク実行のたびに [physics] ログの先頭に出力する。内容を変更した際は必ず
# ここも更新し、環境側のファイルが古い/キャッシュされている疑いを
# ログだけで切り分けられるようにする。
BAKE_HAIR_VERSION = "2026-08-16a (chain parenting now prefers bone-hierarchy-consistent joints: Dijkstra priority is (inconsistent-edge count, 3D distance), so a suspension joint that ties a chain tip back to the waist — e.g. しっぽ吊/腰ベルト６ on Tda Miku Append kai — can no longer steal the tip as a direct child of the anchor and reverse the chain end; such joints now become lateral distance constraints, which is their physical role. Fixes tail/belt tips exported with translations in the wrong parent frame, seen in UE5 as chains stretching to the floor) | previously 2026-07-28n (drape_depth_scale now falls back to the rigid body thickness for bodies whose drape depth could not be measured, instead of dropping their clearance to zero) | previously 2026-07-28m (symmetrize_colliders option: mirror-average left/right collider pairs for baking, since modelling is normally done mirrored and small left/right slips show up as a one-sided skirt bulge; also fixes the report, which ignored exclude_rb and the group mask and therefore counted the chain anchor as a collider) | previously 2026-07-28l (physics_report also estimates mesh-level penetration: clearance measured against the mesh-derived drape depth, so the number reflects what is visible rather than whether the bone itself is clear) | previously 2026-07-28k (the per-ring diagnostics are now opt-in via physics_report) | previously 2026-07-28j (skip_embedded_parent: when a segment starts inside a collider by design — IA seats the skirt chain's parent body 0.052 inside the 下半身1 capsule — the segment test read that as a deep collision and transferred part of it to the first skirt ring every single frame; such a segment is now tested at its child end only) | previously 2026-07-28i (push_preserve_length: after a segment-aware collision push, the particle was re-projected onto the rest_len sphere around its parent, so any contact also stretched a compressed segment to full length — a tiny push became a large outward move. It now keeps the pre-push distance and only rotates about the parent, as the circular-motion design intended) | previously 2026-07-28h (diagnostic: count how often the collision push actually fires per skirt ring and how large it is) | previously 2026-07-28g (diagnostic: sample which collider each skirt ring is closest to during the motion, and at what surface distance, to name what the cloth is actually resting on) | previously 2026-07-28f (collision_velocity_damp: collision push-out moved pos only, so in Verlet every push injected outward velocity; with contact on most frames the skirt was fed outward and never came back — measured as a flat +0.025 waist-ring expansion that vanished with colliders off. prev now moves with the push so the response is fully inelastic) | previously 2026-07-28e (diagnostic: also report which skirt particles are already touching a collider in the rest pose, and the tightest particle-collider pairs, since a permanent contact shows up as a constant ring expansion) | previously 2026-07-28d (diagnostic: report per-ring skirt expansion vs the rest radius each frame, with a 10-slice timeline, to tell contact-driven opening apart from opening that never recovers) | previously 2026-07-28c (margin_rest_clamp: cap the total clearance of each particle-collider pair at the distance actually available in the rest pose, so no margin pushes the cloth outward while the model stands still; needed because the collision-mask fix widened the skirt's collider set to the hip capsules) | previously 2026-07-28b (collision mask semantics fixed: the PMX 16bit field stores which groups a body DOES collide with, Bullet-style; the old code treated a set bit as no-collision and skipped with or, which inverted every group decision) | previously 2026-07-28a (drape_depth_scale: use mesh-measured drape depth per bone as the skirt extra clearance, replacing the rb_size_margin_scale approximation; measured on IA: waist 0.089 / mid 0.118 / hem 0.330 in PMX units, i.e. a 3.7x root-to-hem gradient that a single uniform clearance cannot match) | previously 2026-07-27a (hem_extend_scale: virtually extend tail segments by the tail rigid body's half height for collision, based on inspector finding that tail bones sit at plate centers; substep composition fixed: drag_sub=1-(1-drag)**(1/N) — old drag**(1/N) assumed drag was the retention factor and over-damped activated frames to near-zero momentum — and stiffness now scaled by 1/N like gravity)"

def _sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def _len(a): return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

class Particle:
    __slots__ = ("rb", "bone", "mass", "inv_mass", "parent", "rest_len",
                 "rest_dir_local", "ang_min", "ang_max", "joint_rot", "rest_pos",
                 "kinematic", "group", "collision_mask")
    def __init__(self, rb, bone, mass, kinematic, group=0, collision_mask=0xFFFF):
        self.rb = rb              # physicsGltf.rigidBodies のindex
        self.bone = bone          # node index
        self.mass = mass
        self.inv_mass = 0.0 if (kinematic or mass <= 0) else 1.0/mass
        self.kinematic = kinematic
        self.parent = -1          # 親パーティクルのindex（chain内）
        self.rest_len = 0.0       # 親との静止距離
        self.ang_min = None       # jointの角度制限（ラジアン, x,y,z）
        self.ang_max = None
        self.joint_rot = None     # jointの姿勢(子ボーンのbindローカル基準、quat)。
                                   # ang_min/maxのX/Y/Z軸はこの姿勢で定義されている。
                                   # simulate_step_rigid_chainの軸別クランプでのみ使用。
        self.rest_pos = None      # rest世界座標(glTF)
        self.group = group                      # MMDの非衝突グループ番号(0-15)
        # PMXの16bitフィールド。名前は「非衝突グループフラグ」だが、格納されている
        # ビット=1は「そのグループと衝突する」を意味する(Bulletのcollision filter
        # maskそのもの)。PMXエディタのチェックボックス表示(チェック=非衝突)とは
        # 反転しているので注意。three.jsのmmd-parserはgroupTarget、sabaは
        # m_collisionGroupという名前を使っており、いずれもBulletのmaskへ直接渡す。
        # blender mmd_toolsはimport時に反転して「衝突しないグループ」へ変換している。
        # glTF側のキー名 "noCollisionMask" は互換性のため据え置き(値の意味は同じ)。
        self.collision_mask = collision_mask

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
        p.joint_rot = j.get("rotation")
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
        # set_anchor()内の遅延初期化(getattr)に頼らずここで作る。アンカーが
        # 1つも無いモデルではset_anchorが一度も呼ばれず、rigid_chainパスの
        # state_cloth._anchor_rot 直接参照がAttributeErrorで落ちていた。
        # set_anchorが呼ばれる通常ケースでは挙動・演算結果とも完全に同一。
        self._anchor_rot = {}  # rb -> アンカーボーンのワールド回転(quat)
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
    pos, prev, rest_dir = state.pos, state.prev, state.rest_dir
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
                        allowed_collider_names=None, hem_extra_margin=0.0,
                        hem_extend_scale=0.0,
                        adaptive_substep_threshold=None, adaptive_substep_max_n=4,
                        adaptive_substep_collider_names=None,
                        midpoint_correction=False, midpoint_correction_iters=2,
                        midpoint_correction_margin=0.0,
                        midpoint_correction_collider_names=None,
                        midpoint_correction_samples=1,
                        collision_bounce=0.0,
                        rb_size_margin_scale=0.0,
                        drape_depth_by_bone=None,
                        drape_depth_scale=0.0,
                        drape_probe=True,
                        margin_rest_clamp=True,
                        collision_velocity_damp=1.0,
                        push_preserve_length=True,
                        skip_embedded_parent=True,
                        physics_report=False,
                        symmetrize_colliders=False,
                        lateral_slack_scale=0.0,
                        vertical_spread_scale=0.0,
                        cloth_algorithm="points",
                        rigid_chain_gravity_power=20.0,
                        rigid_chain_stiffness_force=1.0,
                        rigid_body_gravity=9.8,
                        rigid_body_linear_damping=1.0,
                        rigid_body_angular_damping=1.0,
                        rigid_body_warmup_seconds=20.0/30.0,
                        rigid_body_iterations=4,
                        rigid_body_prime_full_motion=False,
                        rigid_body_self_collision_scale=1.0,
                        rigid_body_max_linear_speed=20.0,
                        rigid_body_max_angular_speed=30.0,
                        rigid_body_substeps=2,
                        segment_aware_collision=False,
                        skirt_bone_twist=False):
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
    collision_bounce : コライダーに押し出された時、そのフレーム中に実際に押し
        出された量の合計に比例して、コライダーからより弾かれた位置へ余分に
        押し出す係数(値を上げるほど、体や脚に当たった瞬間に大きく弾むように
        見える)。デフォルト0.0は既存挙動と完全に同じ(prev/posの計算に一切
        影響しない)。1フレームの中で反復拘束解決のために複数回押し出しが
        行われるが、都度オーバーシュートを適用すると呼び出し回数分複利的に
        積み重なり値を大きくすると破綻するため、1フレーム分の押し出し量を
        いったん合計してから、フレームの最後に一度だけ適用する設計。
        gravity_power(重力)・stiffness_force(rest位置へ戻ろうとするバネの
        強さ)とは独立したパラメータで、髪・スカート双方の物理(state_other/
        state_cloth)に同じ値が適用される。詳細はsimulate_step_cloth()内の
        _bounce_accum、resolve_collisions() の accum 引数を参照。
    rb_size_margin_scale : スカート等パネル剛体自身の実サイズ(PMXの形状=箱の
        場合のサイズ_x/y/z のうち最小成分＝厚み方向)を、この倍率で
        collision_margin/hem_extra_margin に上乗せする追加クリアランス。
        デフォルト0.0で無効(既存挙動と完全に同じ)。
        狙い: 現状の押し出しはパーティクル(=ボーン位置)を大きさゼロの点として
        扱っているが、PMX側のスカート剛体は実際には薄い箱として配置されて
        おり、ボーン位置から実メッシュ表面まで箱の厚み分の距離がある。本家
        MMD(Bullet)はこの体積込みで衝突を解くため「表面に密着するのに
        貫通もしない」見た目になる一方、本ツールは一律の固定クリアランス
        だけで近似していたため、値を小さくすると食い込み、大きくすると
        ボーンごと浮いて膨らんで見える、というトレードオフになっていた。
        IAモデルのPMXを実測すると、スカート剛体(形状=箱)の厚み方向サイズは
        腰側で約0.083〜0.085、裾側で約0.122(PMX原単位。scale=0.08の既定
        変換だとglTF単位では約0.0067〜0.0098)と、裾に近いほど厚い傾向が
        確認できた。この値は既存のcollision_margin(既定0.01)・
        hem_extra_margin(実機設定0.03)と同程度の桁であり、突飛に大きい値
        ではない。これは既存のhem_extra_margin(裾だけ追加クリアランス)の
        設計と方向性が一致しており、モデル制作者が同種の配慮を剛体サイズ
        という形で既に作品に織り込んでいた可能性がある。
        本パラメータは、その実サイズをclearanceの参考値としてモデルごとに
        自動的に取り込むための入力で、ボーン本体の押し出し(resolve_collisions)
        と横リングの中間パーティクル補正(_apply_midpoint_correction)の
        両方に、hem_extra_margin と同じく一貫して適用される。
        リング(段)ごとに剛体の厚みが不連続に変わるモデルでは、各パネル
        自身の生の値をそのまま使うとリングの境目に見た目の段差が出ること
        が実機フィードバックで判明したため、各チェーンの根元(生値)→裾
        (生値)を hem_weight と同じ比率で線形補間し、リング間を滑らかに
        繋ぐ(hem_extra_marginの「根元から裾へ連続的に変化」という設計を
        踏襲)。そのため、途中のリングでは実際の剛体サイズそのものではなく
        根元・裾を結ぶ直線上の値が使われる(布らしい滑らかさを優先し、
        個々のリングの厳密な実測値は近似する設計判断)。
        対象は形状=箱(shape=1)のスカート等下半身系(skirt_rbs)剛体のみ。
        カプセル/球で表現された揺れ物には適用しない(該当ケースが少なく、
        「厚み」の解釈がそのままでは成り立たないため安全側で見送り)。
        注意: PMXのサイズ値が半径相当(half-extent)か全体寸法(full-extent)
        かは規格上の解釈がツール依存になりうるため、本パラメータは
        「剛体の最小サイズ成分 × この倍率」という素直な計算に留め、
        1.0を基準にモデル・実機の見た目で上下に調整することを想定する
        (collision_margin/hem_extra_margin と同様、値の妥当性は実測ではなく
        目視で追い込む前提)。
    drape_depth_by_bone / drape_depth_scale : ドレープの落ち込み(メッシュ実測)を
        追加クリアランスとして使う。drape_depth_scale=0.0(デフォルト)で無効。
        drape_depth_by_bone は {ボーンindex: 落ち込み量(PMX原単位)} の辞書で、
        mmd2gltf.drape.measure_drape_depth() が返すものを convert.py 経由で
        受け取る。値は scale を掛けて glTF 単位に変換したうえで
        extra_margin_by_rb として使う(rb_size_margin_scale と同じ配線)。
        狙い: 物理インスペクタでの目視から、スカートボーンが布の外寄り(山側)に
        置かれており、実メッシュはそこからさらに内側へ落ち込んでいることが
        判明した。パーティクル(=ボーン)をコライダーの外に保っても、内側へ
        落ち込んだ布は脚を突き抜けて見える。これが貫入の見かけ上の原因で、
        rb_size_margin_scale で剛体の厚みを何倍かしていたのは、この落ち込みを
        間接的に近似していたことになる。
        IAモデルの実測(PMX原単位・p90)では、落ち込みは腰側リングで平均0.089、
        中段0.118、裾側0.330と、根元から裾へ約3.7倍の勾配があった。一方
        rb_size_margin_scale=3 は全リング一律の約0.29に相当し、腰側では実測の
        3倍以上、裾側では実測にわずかに足りない配分になっていた。実機で
        「3で本家と同程度、5で貫入ゼロだが膨らむ」となったのはこのためと
        考えられる。本パラメータはモデルごとの実測値をそのまま使うので、
        倍率の手調整を前提としない(1.0 = 実測どおり)。
        リング間の段差を避けるため、チェーンに沿って [0.25, 0.5, 0.25] の
        平滑化を1回かける(両端は保持)。rb_size_margin_scale のような根元-裾の
        直線補間ではなく実測の形を残すのは、中間リングの落ち込みが直線上には
        乗らない(IAでは腰0.089→中段0.118→裾0.330)ため。
        drape_depth_scale > 0.0 のときは rb_size_margin_scale 由来の値を
        置き換える(同じ現象を二重に埋めないため。ログにその旨を出力する)。
        静止姿勢クランプ: 実測どおりの深さでも、そのぶん内側に置いた点が
        フレーム0の時点で既にコライダーの内側に入る箇所がある(IA実測では
        36本中4本、いずれも左右の側面=太ももに最も近い列)。そこを無条件に
        押し出すと常時押し出しが働き、避けたかった腰まわりの膨らみが出るため、
        フレーム0の実クリアランス - collision_margin を上限として各剛体ごとに
        頭打ちにする。該当箇所ではモデル自身の布が静止姿勢でカプセルを貫いて
        おり(カプセルが粗い近似であるため)、本家MMDもそこは許容している。
    drape_probe : True(既定)なら、中間パーティクル補正のサンプル点を、ドレープの
        深さぶん内側(体の軸から見た水平の内向き)へ寄せてから衝突を判定する。
        深さを「マージンの上乗せ」ではなく「点の位置」で表現する方式で、
        中間パーティクルが実際のドレープの深い位置に置かれるため、押し出しの
        向きも布の内側基準になる。二重計上を避けるため、この方式のときは
        深さをマージン側には加算しない。drape_depth_scale=0.0(既定)のときは
        drape_extra が空なので、この設定に関わらず従来挙動と完全に同じ。
        False にすると従来どおりマージンへの上乗せとして扱う(比較用)。
        なお hem_extend_scale で終端セグメントを仮想的に延長したのと同じ
        「状態を持たない仮想点」の設計で、パーティクル自体は増やさない。
    collision_velocity_damp : 押し出しが速度を生まないようにする度合い。
        1.0(既定)で完全に打ち消す、0.0で従来どおり。
        背景: resolve_collisions は pos だけを動かす。Verlet積分では速度を
        pos-prev で表すため、押し出した分がそのまま外向きの速度になり、次の
        フレームの慣性に乗ってしまう。毎フレーム接触が起きていると外向きの
        速度が供給され続け、布は押し戻されるどころか外へ送られ続ける。
        実測(IA、7001フレーム): コライダー有りでは腰リングの開きが全区間
        +0.025で平坦(最小でも+0.017、一度も戻らない)、コライダーを外すと
        中央-0.001で±0.03を振動。静止姿勢で食い込んでいるパーティクルは
        36本中1本しかなく、常時食い込みでは説明できなかった。
        押し出した変位と同じだけ prev も動かせば、押し出しは位置だけを直し
        速度には触れない(完全非弾性の法線応答)。collision_bounce は逆に
        prev を戻して弾みを与える機能で、両立する(bounce適用後に打ち消す)。
    symmetrize_colliders : True にすると、左右対のコライダー(「左○○」と「右○○」)
        のボーンから見た相対位置・サイズ・回転を鏡像平均し、ベイクの入力だけを
        左右対称にする(既定 False。extras.mmd に書き出す元データは変更しない)。
        モデル制作は基本ミラー作業なので、左右の食い違いは作者の意図ではなく
        作業上のブレであることが多い。IAでは太ももカプセルのローカルXが
        左-0.193/右+0.147(鏡像なら+0.193)、足が左-0.153/右+0.086と0.046〜0.067
        ずれており、ボーン自体は±1.034で完全対称だった。このずれはドレープの
        マージンと同程度の大きさなので、片側だけスカートが押されて膨らむ。
    physics_report : True にすると、スカートの各リングについて「静止時の半径から
        どれだけ開いたか」「どのコライダーに一番近いか」「押し出しが何%のフレームで
        起きたか」をベイク後にログ出力する(既定 False)。原因の切り分け用で、
        毎フレームの集計ぶん少し遅くなる。
    margin_rest_clamp : True(既定)なら、各(パーティクル, コライダー)の組み合わせに
        ついて、フレーム0での実クリアランスを超えるマージンを使わない。
        collision_margin / hem_extra_margin / extra_margin_by_rb の合計に対して
        効く。考え方は「静止姿勢のモデルは作者が正しく作った状態なのだから、
        どのマージンも静止姿勢で布を押し出してはいけない」。
        背景: PMXのグループ判定を正しく直した結果、スカートの衝突相手が脚だけ
        でなく腰・胴のコライダーにも広がった(IAでは16個)。腰のコライダーは
        スカート上端のすぐ内側にあり、実測クリアランスはリング0で中央0.0156・
        最小0.0065(glTF単位)しかない。ここに既定のcollision_margin=0.01と
        rb_size_margin_scale=1.0相当の0.0077が乗ると合計0.018となり、静止姿勢
        から腰リング全周が押し出されて、スカートが傘型に開いてしまう。
        ペア単位で上限を持たせるのが要点で、同じ腰のパーティクルでも太ももに
        対しては0.024程度の余裕があるため、脚への当たりは甘くならない。
        False にすると従来どおり上限なし(比較用)。
    lateral_slack_scale : スカートの横リング(隣接パネル間、lateral)の距離拘束に
        「遊び」を持たせる倍率。デフォルト0.0で無効(既存の常にrest_lenぴったり
        へ戻すハード等式拘束のまま、既存挙動と完全に同じ)。
        狙い: PMXの実ジョイントデータを調べたところ、このモデルの横リング
        ジョイントはバネ定数が完全に0で、移動制限(linearLimitMin/Max)が
        ±0.017前後(リングにより異なり裾側リングほど大きい傾向)、回転制限
        ±30°という「範囲内は完全フリー、限界で硬く止まる」設計だった。
        一方、本ソルバーはこれを常に厳密なrest_len等式拘束として扱っており、
        伸縮が一切許されないため、フレアスカートが折り紙のように硬く見える
        一因になっていた(実機フィードバックで確認)。
        本パラメータを1.0にすると、各横エッジについてextract_chains_bfsが
        該当ジョイントのlinearLimitMin/Maxから概算した「遊び」量(joint_slack、
        軸ごとの制限値の絶対値平均)をそのまま使い、距離が
        [rest_len-遊び, rest_len+遊び] の範囲内なら一切補正しない(本家の
        バネ定数0ジョイントと同じ「範囲内フリー」を模す)。範囲外に出た
        フレームだけ、範囲の境界へ押し戻す(rest_lenそのものへは戻さない)。
        joint_slackが0(縦チェーンなど元々遊びが無いジョイント)の辺は
        rb_size_margin_scale同様、常にtarget=rest_lenとなり無効化と同じ。
        1.0を基準に、硬さの残り具合を見ながら調整することを想定する
        (値の妥当性は実測ではなく目視で追い込む前提、他パラメータと同様)。
    vertical_spread_scale : コライダーに当たって押し出された1パーティクルの
        変位を、同じチェーン内で縦方向に隣接する1つ上・1つ下のパーティクル
        (kinematicアンカーは対象外)へもこの倍率だけ分配する。デフォルト0.0
        で無効(既存挙動と完全に同じ)。
        狙い: 本ソルバーは「点」の集まりで、コライダーに当たった段だけが
        その場で平行移動して押し出されるため、隣の段との間に不連続な段差
        (輪切り)が出る。実機フィードバックで、片足を上げるなど脚が
        大きく傾くポーズでスカートに輪切りの段が目立つ一方、本家MMDでは
        腰元から裾まで滑らかな曲線を描くと報告された。本家は縦ジョイントの
        移動が完全固定な代わりに回転が非常に自由(実測X軸±80°)で、パネルが
        平行移動でなく脚の角度に追従して傾くため連続的に見える。本パラメータ
        は、剛体+回転ジョイントへの全面的な書き換えを行わずに、この「1段
        だけ飛び出る」効果を緩和する近似策として、押し出し変位を隣接パー
        ティクルへも滲み出させる(ラプラシアン平滑化に近い後処理)。
        実装上は、そのフレーム中にresolve_collisionsが実際に押し出した
        変位の合計(collision_bounceと同じaccum機構を共用)を使い、フレーム
        につき一度だけ・1ホップ先の隣接パーティクルにのみ適用する(多段階の
        伝播はしない)。適用は縦方向の長さ拘束の再クランプ(修正3)より前に
        行うため、分配した後の位置は再クランプでrest_lenへ戻され、伸縮では
        なく「向き(傾き)」の変化として隣接パーティクルに伝わる。
        1.0を基準に、輪切りの残り具合を見ながら調整することを想定する
        (他の実験的パラメータと同様、値の妥当性は目視で追い込む前提)。
    cloth_algorithm : スカート(skirt_rbs)の解法を選ぶ。デフォルト"points"は
        既存の点ベース方式(simulate_step_cloth、本ファイルの主力実装、
        collision_margin/hem_extra_margin/rb_size_margin_scale/
        lateral_slack_scale/vertical_spread_scale が全て効く)。
        "rigid_chain"は実験的な板(剛体チェーン)方式で、各段の「姿勢」を
        独立した物理状態として持ち、慣性・重力トルク・stiffness緩和・
        本家ジョイント実測の軸別角度制限(X/Y/Z、joint_rot基準)で駆動する
        (simulate_step_rigid_chain参照)。横リングは"points"と同じ
        _apply_lateral_constraint(lateral_slack_scale込み)を共有。
        現時点ではコライダー衝突・適応サブステップ・中間パーティクル補正は
        rigid_chainに未対応(縦の回転駆動＋横リングのみ)。"points"の
        gravity_power/stiffness_forceとは数値の意味が違う(位置ベースの
        線形変位ではなく回転角[ラジアン]で効く)ため、rigid_chain選択時は
        rigid_chain_gravity_power/rigid_chain_stiffness_force の方を使う。
    rigid_chain_gravity_power : cloth_algorithm="rigid_chain"専用の重力の
        強さ(1フレームあたりの回転角[ラジアン]の目安、内部でdt倍される)。
        既定20.0は実データでの単体検証(裾の偏差角が本家ジョイントのX軸
        制限=リングにより20〜80°で頭打ちになる)から得た目安値。
    rigid_chain_stiffness_force : cloth_algorithm="rigid_chain"専用の
        stiffness(1フレームで偏差をrestへ戻す割合、内部でdt倍後0〜1に
        クランプ)。既定1.0。値の妥当性は目視で追い込む前提。
    cloth_algorithm="rigid_body" (Stage 3、実験的): "rigid_chain"(姿勢のみ)
        では回転しても遠心力で広がらないという実機フィードバックを受け、
        Bullet(本家)の設計(m_linearVelocity/m_angularVelocityを両方
        独立に積分)を参考に、並進速度・角速度を両方持つ本物の剛体
        チェーンとして作り直したもの(RigidBodyXPBD・_rb_step参照)。
        質量・慣性テンソルはPMX剛体の実サイズ・実質量から計算し、
        本家ジョイント実測の軸別角度制限も使う。横リングは既存の
        lateral_slack_scaleと同じ「遊び」概念を剛体の距離拘束として流用。
        コライダー衝突は重心のみを対象にした簡易版(剛体の厚みは考慮せず
        並進のみで押し出す、回転は変えない)を実装済み。適応サブステップ・
        中間パーティクル補正は現時点で未対応。
    rigid_body_gravity : cloth_algorithm="rigid_body"専用の重力加速度の
        大きさ(gravity_dir方向、m/s^2相当。既存のgravity_powerとは単位が
        違う実数値の物理量)。既定9.8(地球の重力加速度)。
    rigid_body_linear_damping, rigid_body_angular_damping : PMXの各スカート
        剛体が実際に持つ移動減衰・回転減衰の値に掛ける倍率(既定1.0=モデルの
        実測値をそのまま使う)。本家のbtRigidBody::applyDamping(1秒あたりの
        除去割合damping[0..1]に対し (1-damping)^dt で減衰させる指数関数式)
        と同じ式を使う。これが無いと(摩擦の無い振り子のように)いつまでも
        大きく揺れ続ける。実測では本モデルのスカートは0.9/0.9(以前この
        関数の既定値としていた固定0.1/0.3は、モデルによらず全剛体に一律で
        当てはめていた誤りで、減衰不足の原因になっていた。実機フィード
        バックで発覚し修正)。
        【重要な過去のバグ】(1-damping*dt*30)という線形近似の式を一時期
        使っていたが、damping=0.9のような大きい値では正しい残存率92.6%/
        フレームに対し誤って10%/フレームまで減衰させてしまい、剛体が
        実質静止した状態に固まる不具合があった(実機フィードバック
        「スカートの大半が初期ポーズ位置に残った状態」で発覚、指数関数式
        へ修正済み)。
    rigid_body_warmup_seconds : cloth_algorithm="rigid_body"専用の助走時間
        (秒、frame0の姿勢を保持したまま静かに重力で落ち着かせる時間)。
        既定20/30秒(既存のwarmup=20フレームと同じ、後方互換)。bind pose
        から自然に垂れるまでの遷移が急に見える場合、大きくすると効果が
        ある(実機フィードバックで確認)。"points"/"rigid_chain"のwarmup
        (既存の変数warmup、20フレーム固定)には影響しない。
    rigid_body_iterations : cloth_algorithm="rigid_body"専用の反復拘束解決
        (Gauss-Seidel)の回数。既定4。PMXの実測減衰値(このIAモデルのスカート
        は0.9/0.9)をそのまま使うと、既定4回では反復が収束しきらずリングが
        崩れる不具合が実機フィードバックで発覚(減衰を弱めれば見た目は良く
        なるが、それはモデルの実測値を無視した当て推量になってしまうため
        避けるべきと判断)。反復回数を増やして収束精度を上げる方向で対応する。
    rigid_body_prime_full_motion : cloth_algorithm="rigid_body"専用。Trueに
        すると、本番の記録を始める前に全モーション(num_frames分)を一度
        通しで動かし(出力は記録しない)、その「落ち着いた」物理状態から
        frame0の記録をやり直す2パス方式になる。本家VMDビューアの実機
        フィードバックで判明した仕様(初回再生はframe0の物理がぐちゃぐちゃ
        だが、一度最後まで再生してframe0に戻ると、体のボーンはキーフレーム
        通りに戻る一方スカート・髪は最後に落ち着いた状態を引き継ぐため
        2回目から狙った動きになる)を模したもの。既定Falseで無効(既存挙動
        と同じ、1パスのみ)。有効にすると変換時間が概ね倍になる。
    rigid_body_self_collision_scale : スカート自己衝突(_rb_solve_min_
        separation)の最小距離を、両剛体のhalf_w(サイズのX成分)の合計の
        何倍にするかの係数。既定1.0(=half_wの合計そのまま)。
        当初は「bind poseの時点で既に隣接パネルの大半が衝突判定される
        (実測: IAモデルでは横リング36本全てが違反、最も厳しいペアで距離が
        しきい値の39.5%)ことが、スカートが初期ポーズ位置に固まって動けなく
        なる不具合の原因」と誤って判断し0.3まで下げたが、実際の原因は
        _rb_integrate_predictの減衰式バグ(下記参照)だった。バグ修正後は
        1.0の方が明らかに良い結果(方位角の隙間が全体を通して健全な範囲に
        近づく)になることを実機フィードバックで確認済み。0.3等に下げる
        必要は無いが、他モデルでは調整が必要な場合がある。
    rigid_body_max_linear_speed, rigid_body_max_angular_speed : 各剛体の
        並進速度(単位/秒)・角速度(ラジアン/秒)の上限クランプ。既定20.0/30.0。
        本家Bullet(btRigidBody::integrateVelocities)にも角速度の上限
        (MAX_ANGVEL=π/2、1ステップでの過大な回転を防ぐ安全装置)があり、
        この実装には無かったため追加した。アンカーが1フレームで大きく
        飛ぶ場面(モーション冒頭、rigid_body_prime_full_motion使用時の
        priming pass→本番パス切り替わり等)で生じる過大な暗黙速度が
        後続フレームに伝播し続け不安定化する事例(MMD本家にも、モーション
        読み込み時の急変を物理エンジンが「衝撃」と捉え一部モデルが
        自己持続的に崩れたまま戻らないという既知の未解決問題があるとの
        文献調査結果と一致)への対策。
    rigid_body_substeps : cloth_algorithm="rigid_body"専用。1フレーム
        (1/fps秒)を何回のサブステップに分けて解くか。既定2。ユーザー情報
        「本家VMDビューアは60FPSで動いている」(このツールの既定30FPSの
        2倍の時間解像度)を受けて追加。反復拘束解決(rigid_body_iterations)
        の回数を増やすのではなく、1ステップあたりの時間刻みそのものを
        小さくすることで、同じ反復回数でも収束しやすくする狙い。アンカー
        (前フレームとの間)は位置線形補間・姿勢球面補間(既存の適応サブ
        ステップ機能が使う_lerp_anchorを共有)で滑らかに繋ぐ。1にすると
        従来通り1フレーム=1ステップ。
    segment_aware_collision : cloth_algorithm="points"専用(スカートの
        state_clothにのみ適用、髪等のstate_otherには影響しない)。既定
        False。Trueにすると、コライダー衝突の押し出しをパーティクル単体
        でなく「親→子の区間」に対して判定する(resolve_collisions_segments、
        resolve_collisionsの代わりに使われる)。従来はある1点だけが衝突
        検出されるとその点だけが飛び出し、上下の段には何も伝わらないため
        「階段形状」(片足を上げるポーズでの輪切り感)の一因になっていた
        (実機フィードバック「本来ならぶつかった点のベクトルがスカート側の
        点に転写されるべき」を受けて実装)。衝突点が区間の途中(親端・子端
        ちょうどでない)にあるときは、その位置に応じて押し出しを親・子へ
        自然に按分する。t=0/1(端点そのもの)のときは既存のresolve_
        collisionsと完全に同じ結果になる退化ケースとして一致する。
        vertical_spread_scale(結果を後から一律の割合で隣に配る仕組み)とは
        独立で併用も可能だが、本機能は衝突の実位置に応じた按分のため、
        vertical_spread_scaleを弱める・0にしても階段対策の効果が期待できる。
    skirt_bone_twist : cloth_algorithm="points"専用(スカートのみ、髪には
        影響しない)。既定False(既存挙動と同じ、スカートボーンの回転出力を
        常に恒等=親と同じ向きに固定)。Trueにすると、simulate_step_cloth
        が既に安全クランプ(q_slerp_toward、前フレームから最大60度までしか
        許容しない)を適用した本物の回転値をそのまま使う。位置(translation)
        は元々厳密に一致させているので影響を受けず、回転(メッシュのツイスト
        表現)だけが変わる。実機フィードバック(「区間の中点あたりに横方向へ
        押し出されたような段ができる、ボーン自体の傾きではない」)を受けて
        追加: 恒等固定だと区間がどれだけ傾いているかの情報が全部失われ、
        スキニングの補間が実際の傾きと食い違って段になっていたと判断。
    hem_extra_margin : 各スカートチェーンの根元(腰側)から裾にかけて、0→この値まで
        滑らかに線形補間しながら collision_margin へ上乗せする追加クリアランス
        (根元=0、裾=hem_extra_margin全量)。デフォルト0.0で無効(既存挙動と
        完全に同じ)。太もも等へのメッシュ面の見た目上の食い込みは裾で起きやすい
        一方、collision_margin自体を全体で上げると腰に近いセグメントが押し
        出されて「傘化」に近づいてしまう(実測: margin 0.08でほぼ元の傘化
        相当まで戻る)ため、根元は完全に不変のまま裾へ向けて安全にクリアランスを
        稼ぐための逃げ道。裾の1パーティクルだけをON/OFFする段差ではなく、
        チェーンの深さに比例して連続的に変化するので不自然な継ぎ目が出ない。
    hem_extend_scale : 裾延長。スカート各チェーンの終端セグメントを、最下段
        剛体(形状=箱)の半高さ×この倍率だけ子端方向へ仮想的に延長して衝突
        評価する(resolve_collisions_segments専用。segment_aware_collision=True
        とセットで使う)。実機の物理インスペクターでの観察「縦ボーンの終端は
        剛体プレートの中央にある」に基づく: 本家では最下段の箱が終端ボーンの
        下へ半高さぶんはみ出して干渉域を持つが、リング(パーティクル/線分)
        評価にはその領域が存在せず、同じ姿勢でも脚が一段深く入ってから
        押し返し始める。延長量はPMXの実寸(size[1]。PMXの箱サイズは半値=
        Bulletのhalf extentsなので、そのまま半高さ)から取るため、
        rb_size_margin_scale等と同じ「モデルにある数値を使う」設計。
        1.0でちょうど本家の箱の底に相当する位置まで届く。延長部でのヒットは
        「裾の縁が押された」とみなし子端全量(t=1.0)として転写する。
        デフォルト0.0で無効(既存挙動とビット単位で完全に同じ)。
    adaptive_substep_threshold : 適応サブステップの発動閾値(コライダー半径に
        対する比率)。フレーム間の「布アンカー変位＋近傍コライダー変位」の
        合計(三角不等式による安全側の上限見積もり)が、最も近いコライダーの
        半径のこの倍率を超えたフレームだけ、布ソルバ(スカート等)をNステップに
        分割して呼ぶ(髪など布以外の物理は常にN=1で無改造)。デフォルトNoneは
        機構自体が無効で、常にN=1(既存挙動とビット単位で完全に同じ)。実測の
        目安は フェーズA計測(脚コライダーのみ) でぽんぷ/IAとも中央値0.36〜0.40、
        p95が1.4〜1.8程度だったため、0.5〜0.75あたりが「明らかに怪しいフレーム
        だけを拾う」現実的な初期値の候補(発動率2〜4割)。値を大きくするほど
        発動フレームが減り安全側(=無効化に近づく)、小さくするほど発動が増え
        安全だがベイクが遅くなる方向なので、どちらに振っても破綻はしない。
    adaptive_substep_max_n : 適応サブステップの分割数の上限。デフォルト4。
    adaptive_substep_collider_names : 適応サブステップの発動判定で見る
        コライダー剛体名の集合(set/list)。指定すると、この名前に一致する
        コライダーの移動だけを発動判定に使う(例: 脚だけに絞って腕の素早い
        動きを無視する)。デフォルトNoneなら衝突判定に使われている全コライダー
        を見る。衝突判定そのもの(resolve_collisions)には影響しない、発動
        タイミングの計算だけに使う絞り込み。
    midpoint_correction : 隣接するチェーン同士を結ぶ横リングエッジ(lateral、
        同じ高さの輪)上のサンプル点(既定では中点1点、midpoint_correction_samples
        で増やせる)がコライダーに食い込んでいたら、その両端の実ボーン位置を
        少しだけ押し出して緩和する追加パス。デフォルトFalseで無効(既存挙動と
        完全に同じ)。ボーン(関節点)自体は既に押し出し済みでも、ボーンと
        ボーンの間のメッシュ面がコライダー体積を視覚的に横切ってしまう現象
        (ボーンレベルの衝突判定では原理的に検知できない)を近似的に緩和する。
        チェーン内の縦方向(腰→裾)のセグメントは対象にしない(縦方向を押すと
        1本のズレが裾側全体に伝わり、傘のように広がってしまうため)。押し出し
        方向はコライダー表面法線の3D方向。押し出し後は横エッジ長と縦方向の
        セグメント長をrest_lenへ戻す「念押し」も行うので、チェーンが
        伸び縮みすることはない。hem_extra_margin が指定されている場合は、
        ボーン本体の衝突判定と同じ「根元0→裾で全量」の深さ比例で、横リング
        エッジのクリアランスにも一貫して上乗せされる(エッジの深さは両端
        パーティクルの hem_weight の平均。hem_extra_margin=0なら無効)。
        検証の結果、
        貫入の深さ・面積とも6〜7割程度は減るが、「貫入が発生するフレーム数」
        自体はほぼ変わらない(浅く広く残る)。あくまで緩和策であり、メッシュ面の
        完全な非貫入を保証するものではない。適応サブステップが発動している
        フレームでは、サブステップ1回ごと(N回)に補正をかける(検証の結果、
        フレーム終わりに1回だけ補正するより面積・貫入フレーム数の両方で
        わずかに有利だったため)。
    midpoint_correction_iters : 中間パーティクル補正の反復回数。デフォルト2。
        コスト増は小さい(実測: 反復6回でも全体の1割程度増)。
    midpoint_correction_margin : 中間パーティクル補正でのクリアランス
        (collision_marginとは別扱い)。デフォルト0.0。
    midpoint_correction_samples : 各セグメント上でチェックするサンプル点の数。
        デフォルト1(中点のみ、t=0.5)。2以上にすると、セグメントをN+1等分した
        内側のN点(例: samples=2ならt=1/3, 2/3)をそれぞれチェックし、各サンプル
        点での押し出しをt(親側からの距離の割合)で親子に案分して適用する
        (親がkinematicアンカーの場合は常に子へ全量)。値を増やすほど、より
        細かい範囲の食い込みを拾えるが、その分コストは線形に増える。
    midpoint_correction_collider_names : 中間パーティクル補正で見るコライダー
        剛体名の集合。デフォルトNoneなら衝突判定に使われている全コライダーを
        見る。adaptive_substep_collider_namesと同じ集合を渡すのが典型的な
        使い方(脚だけに絞る等)。
    戻り値       : { node_index: [ (x,y,z,w), ... num_frames ] }（glTF局所回転）
    """
    nodes = gltf_json["nodes"]
    print("  [physics] bake_hair.py version: %s" % BAKE_HAIR_VERSION)
    bwm = compute_bone_world_matrices(gltf_json)
    _bone_parents = [-1] * len(nodes)
    for _i, _nd in enumerate(nodes):
        for _c in _nd.get("children", []):
            _bone_parents[_c] = _i
    chains, parts, excluded, lateral = extract_chains_bfs(
        physics_gltf, bwm, only_names=only_names, bone_parents=_bone_parents)
    rbs_names_all = physics_gltf["rigidBodies"]
    _push_report = {}   # 計測用(ウォームアップ中の分は本計測前にリセットする)
    if force_no_collision_names:
        _force_set = set(force_no_collision_names)
        _forced = 0
        for p in parts.values():
            if rbs_names_all[p.rb]["name"] in _force_set:
                p.collision_mask = 0     # 0 = どのグループとも衝突しない
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
            _mask = rb.get("noCollisionMask", 0xFFFF)
            if force_no_collision_names and rb["name"] in force_no_collision_names:
                _mask = 0     # 0 = どのグループとも衝突しない
            colliders_def.append((rb["shape"], path, rb["size"],
                                  rb["position"], rb["rotation"],
                                  rb.get("group", 0), _mask, _rbi))

    if symmetrize_colliders:
        # 左右対のコライダー(「左○○」と「右○○」)の、ボーンから見た相対位置・
        # サイズ・回転を鏡像平均して左右対称にする。
        # 背景: モデル制作は基本ミラー作業なので、左右の食い違いは作者の意図では
        # なく作業上のブレであることが多い。IAでは太ももカプセルのローカルXが
        # 左-0.193/右+0.147(鏡像なら+0.193)と0.046ずれており、足も0.067ずれて
        # いた(ボーン自体は±1.034で完全対称)。この差はドレープのマージンと
        # 同程度の大きさになるため、片側だけスカートが押されて膨らむ。
        # extras.mmd に書き出す元データは変更しない(ベイクの入力だけを直す)。
        _by_name = {}
        for _i, _c in enumerate(colliders_def):
            _nm = rbs_names_all[_c[-1]].get("name", "")
            _by_name[_nm] = _i
        _done = set()
        _nsym = 0
        for _nm, _i in list(_by_name.items()):
            if not _nm.startswith("左") or _i in _done:
                continue
            _j = _by_name.get("右" + _nm[1:])
            if _j is None or _j in _done:
                continue
            _L, _R = colliders_def[_i], colliders_def[_j]
            if _L[0] != _R[0]:
                continue
            _lp, _rp = _L[3], _R[3]
            _ls, _rs = _L[2], _R[2]
            # 位置: xは符号を保って絶対値を平均、y/zはそのまま平均
            _ax = (abs(_lp[0]) + abs(_rp[0])) * 0.5
            _ay = (_lp[1] + _rp[1]) * 0.5
            _az = (_lp[2] + _rp[2]) * 0.5
            _npL = (_ax if _lp[0] >= 0 else -_ax, _ay, _az)
            _npR = (_ax if _rp[0] >= 0 else -_ax, _ay, _az)
            _ns = [ (_ls[_k] + _rs[_k]) * 0.5 for _k in range(min(len(_ls), len(_rs))) ]
            # 回転: 右をX面で鏡像((x,y,z,w)->(x,-y,-z,w))してから平均し、右へ戻す
            _lr, _rr = _L[4], _R[4]
            _mr = (_rr[0], -_rr[1], -_rr[2], _rr[3])
            if (_lr[0]*_mr[0] + _lr[1]*_mr[1] + _lr[2]*_mr[2] + _lr[3]*_mr[3]) < 0.0:
                _mr = (-_mr[0], -_mr[1], -_mr[2], -_mr[3])
            _q = [(_lr[_k] + _mr[_k]) * 0.5 for _k in range(4)]
            _qn = math.sqrt(sum(_v * _v for _v in _q)) or 1.0
            _q = [_v / _qn for _v in _q]
            _qR = (_q[0], -_q[1], -_q[2], _q[3])
            colliders_def[_i] = (_L[0], _L[1], _ns, _npL, tuple(_q), _L[5], _L[6], _L[7])
            colliders_def[_j] = (_R[0], _R[1], _ns, _npR, _qR, _R[5], _R[6], _R[7])
            _done.add(_i); _done.add(_j)
            _nsym += 1
        if _nsym:
            print("  [physics] symmetrize_colliders: %d left/right collider pair(s) "
                  "mirror-averaged for baking (source data unchanged)" % _nsym)

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
    # 縦方向の隣接パーティクル(1ホップのみ、kinematicアンカーは含めない)。
    # コライダーに当たって押し出された1段だけが飛び出て輪切りのように
    # 見える(実機フィードバック: 片足を上げたポーズで顕著)問題への対策
    # vertical_spread_scale で使う。同じチェーン内で前後1つずつを保持。
    vertical_neighbors = {}
    for ch in chains:
        dyn = [p for p in ch.particles if p.rb in skirt_rbs and not p.kinematic]
        k = len(dyn)
        if k == 0:
            continue
        for i, p in enumerate(dyn):
            hem_weight[p.rb] = 1.0 if k == 1 else i / (k - 1)
            nbs = []
            if i > 0:
                nbs.append(dyn[i - 1].rb)
            if i < k - 1:
                nbs.append(dyn[i + 1].rb)
            vertical_neighbors[p.rb] = nbs

    # 剛体自身の実サイズ(箱の厚み)を追加クリアランスとして取り込む。
    # rb_size_margin_scale=0.0(デフォルト)なら空辞書のままで既存挙動と完全に同じ。
    # 対象は skirt_rbs のうち形状=箱(shape=1)のもののみ。サイズ_x/y/z の
    # うち最小成分を「厚み方向」とみなし(パネルの回転で押し出し軸がどの
    # ローカル軸に来るかはモデル依存のため、向きに依らない安全な近似として
    # 最小成分を使う)、この倍率を掛けた値を rb ごとに保持する。
    # 各パネル自身の生の値をそのまま使うと、リング(段)ごとに箱の厚みが
    # 不連続に変わるモデルでは、リングの境目に見た目の「段差」が出てしまう
    # (実機フィードバックで確認: スカートが布らしくない段差状になる)。
    # hem_extra_margin と同じ考え方で、チェーンの根元(生値)→裾(生値)を
    # hem_weight で線形補間し、リング間を滑らかに繋ぐ。
    rb_size_extra = {}
    if rb_size_margin_scale > 0.0:
        _raw_size = {}
        for _rbi in skirt_rbs:
            if not (0 <= _rbi < len(rbs_names_all)):
                continue
            _rbdef = rbs_names_all[_rbi]
            if _rbdef.get("shape") != 1:   # 1=箱 以外は対象外
                continue
            _sz = [s for s in _rbdef.get("size", ()) if s > 0]
            if not _sz:
                continue
            _raw_size[_rbi] = min(_sz) * rb_size_margin_scale
        for ch in chains:
            dyn = [p for p in ch.particles if p.rb in skirt_rbs and not p.kinematic]
            _dyn_raw = [(p, _raw_size[p.rb]) for p in dyn if p.rb in _raw_size]
            if not _dyn_raw:
                continue
            _root_v = _dyn_raw[0][1]
            _hem_v = _dyn_raw[-1][1]
            for p in dyn:
                if p.rb not in _raw_size:
                    continue
                _w = hem_weight.get(p.rb, 0.0)
                rb_size_extra[p.rb] = _root_v + (_hem_v - _root_v) * _w
        if rb_size_extra:
            _vals = list(rb_size_extra.values())
            print("  [physics] rb_size_margin_scale=%.3f: %d skirt rigid "
                  "body box(es) contributing extra margin (min=%.4f, max=%.4f, "
                  "mean=%.4f; smoothed root->hem per chain to avoid ring-to-ring "
                  "steps)"
                  % (rb_size_margin_scale, len(rb_size_extra),
                     min(_vals), max(_vals), sum(_vals) / len(_vals)))

    # 静止姿勢クランプ用の表: {(パーティクルのrb, コライダーのrb): 静止時クリアランス}
    # 考え方は単純で、「静止姿勢のモデルは作者が正しく作った状態なのだから、
    # どのマージンも静止姿勢で布を押し出してはいけない」。フレーム0で実際に
    # 空いている距離を、その組み合わせで使えるマージンの上限とする。
    # ペア単位にするのが要点で、腰のパーティクルは下半身カプセルに対しては
    # わずかな余裕しか無いが、太ももに対しては十分な余裕がある。一律に絞ると
    # 脚への当たりまで甘くなるため、相手ごとに上限を持たせる。
    _pair_cap = None
    if margin_rest_clamp:
        _pair_cap = {}
        _cols0 = colliders_at(0)
        for ch in chains:
            for p in ch.particles:
                if p.kinematic or not (0 <= p.bone < len(nodes)):
                    continue
                _rp = bone_world_pos(p.bone, 0)
                for _col in _cols0:
                    _crbi = _col[-1]
                    if allowed_collider_rbi is not None:
                        if _crbi not in allowed_collider_rbi:
                            continue
                    elif not ((_col[-2] & (1 << p.group)) and (p.collision_mask & (1 << _col[-3]))):
                        continue
                    if _col[0] == "sphere":
                        _C, _rad = _col[1], _col[2]
                    else:
                        _C = _closest_on_segment(_rp, _col[1], _col[2])
                        _rad = _col[3]
                    _pair_cap[(p.rb, _crbi)] = max(0.0, _len(_sub(_rp, _C)) - _rad)
        if _pair_cap:
            _cv = list(_pair_cap.values())
            print("  [physics] margin_rest_clamp: %d particle-collider pair(s), "
                  "rest clearance min=%.4f, median=%.4f, max=%.4f"
                  % (len(_cv), min(_cv), sorted(_cv)[len(_cv) // 2], max(_cv)))
            # どのパーティクルが静止姿勢で「もう触っている」かを出す。
            # ここが多いと、そのぶんは毎フレーム押し出され続ける＝開いたままになる。
            _tight = {}
            _tight_rb = {}
            if not physics_report:
                _pair_cap_items = []
            else:
                _pair_cap_items = list(_pair_cap.items())
            for (_prb, _crbi), _room in _pair_cap_items:
                if _prb not in skirt_rbs or _room > 1e-6:
                    continue
                _cn = rbs_names_all[_crbi]["name"] if 0 <= _crbi < len(rbs_names_all) else str(_crbi)
                _tight[_cn] = _tight.get(_cn, 0) + 1
                _tight_rb[_prb] = min(_tight_rb.get(_prb, 9e9), _room)
            if _tight:
                print("  [physics]   %d skirt particle(s) already touch a collider in the rest "
                      "pose (clearance <= 0); by collider: %s"
                      % (len(_tight_rb),
                         ", ".join("%s x%d" % (k, v) for k, v in
                                   sorted(_tight.items(), key=lambda t: -t[1])[:6])))
            # 各スカートパーティクルについて、一番きつい相手との余裕
            _minroom = {}
            for (_prb, _crbi), _room in _pair_cap_items:
                if _prb not in skirt_rbs:
                    continue
                if _prb not in _minroom or _room < _minroom[_prb][0]:
                    _minroom[_prb] = (_room, _crbi)
            if _minroom:
                _mr = sorted(_minroom.items(), key=lambda t: t[1][0])[:8]
                print("  [physics]   tightest skirt particles at rest: %s"
                      % ", ".join("%s->%s %.4f"
                                  % (rbs_names_all[k]["name"],
                                     rbs_names_all[v[1]]["name"], v[0]) for k, v in _mr))

    # ドレープの落ち込み(メッシュ実測)を追加クリアランスとして使う。
    _drape_active = False
    # drape_depth_scale=0.0(デフォルト)なら何もしない=既存挙動と完全に同じ。
    # 剛体の厚み×倍率(rb_size_margin_scale)は同じ現象を間接的に近似したもの
    # なので、こちらが有効なときは置き換える(二重に埋めない)。
    if drape_depth_scale > 0.0 and drape_depth_by_bone:
        _drape_raw = {}
        for ch in chains:
            for p in ch.particles:
                if p.kinematic or p.rb not in skirt_rbs:
                    continue
                _dv = drape_depth_by_bone.get(p.bone)
                if _dv is None or _dv <= 0.0:
                    continue
                # PMX原単位 → glTF単位
                _drape_raw[p.rb] = _dv * scale * drape_depth_scale
        drape_extra = {}
        for ch in chains:
            dyn = [p for p in ch.particles if p.rb in skirt_rbs and not p.kinematic]
            _seq = [p for p in dyn if p.rb in _drape_raw]
            if not _seq:
                continue
            _v = [_drape_raw[p.rb] for p in _seq]
            # リング間の段差を避けるため [0.25, 0.5, 0.25] で1回平滑化(両端は保持)
            _sm = list(_v)
            for i in range(1, len(_v) - 1):
                _sm[i] = _v[i - 1] * 0.25 + _v[i] * 0.5 + _v[i + 1] * 0.25
            for p, val in zip(_seq, _sm):
                drape_extra[p.rb] = val
        # 静止姿勢クランプ。実測どおりの深さでも、そのぶん内側に置いた点が
        # 最初からコライダーの中に入ってしまう箇所がある(IAでは36本中4本、
        # いずれも左右の側面=太ももに最も近い列)。そこを無条件に押し出すと
        # 常時押し出しが働き、避けたかった腰まわりの膨らみが出る。
        # 上で作った _pair_cap(フレーム0の実クリアランス)の、その剛体に対する
        # 最小値で上限を掛ける。
        _clamped = 0
        if _pair_cap:
            _room_by_rb = {}
            for (_prb, _crbi), _room in _pair_cap.items():
                if _prb in drape_extra:
                    if _prb not in _room_by_rb or _room < _room_by_rb[_prb]:
                        _room_by_rb[_prb] = _room
            for _rbi in list(drape_extra):
                _room = _room_by_rb.get(_rbi)
                if _room is None:
                    continue
                _lim = max(0.0, _room - max(collision_margin, 0.0))
                if drape_extra[_rbi] > _lim:
                    drape_extra[_rbi] = _lim
                    _clamped += 1

        if drape_extra:
            _drape_active = True
            _vals = list(drape_extra.values())
            print("  [physics] drape_depth_scale=%.3f: %d skirt rigid body/bodies "
                  "using mesh-measured drape depth as extra margin "
                  "(min=%.4f, max=%.4f, mean=%.4f; smoothed along chain)"
                  % (drape_depth_scale, len(drape_extra),
                     min(_vals), max(_vals), sum(_vals) / len(_vals)))
            if _clamped:
                print("  [physics] drape depth clamped on %d rigid body/bodies "
                      "(the inward point would already be inside a collider at rest)"
                      % _clamped)
            # ドレープ深さが測れなかった剛体(その骨に支配される頂点が少ない等)は
            # 剛体の厚み(rb_size_margin_scale)の値を残す。まるごと置き換えると、
            # 測れなかったぶんがクリアランス0になって貫通が増える
            # (ぽんぷは48本中28本しか測れず、置き換え版では腰側の貫通が
            #  3.0%→4.1%と悪化していた)。
            _kept = 0
            for _rbi, _v in rb_size_extra.items():
                if _rbi not in drape_extra:
                    drape_extra[_rbi] = _v
                    _kept += 1
            if rb_size_extra:
                print("  [physics] note: rb_size_margin_scale values are superseded "
                      "by drape_depth_scale where the drape depth could be measured"
                      + (" (%d body/bodies keep the rigid body thickness)" % _kept
                         if _kept else ""))
            rb_size_extra = drape_extra

    # 裾延長(hem_extend_scale): 実機の物理インスペクター観察「縦ボーンの終端は
    # 剛体プレートの中央」に基づき、最下段剛体が終端パーティクルの下へはみ出す
    # 半高さぶんだけ、終端セグメントを仮想延長して衝突評価する(適用は
    # resolve_collisions_segments側。segment_aware_collision=True時のみ効く)。
    # 値はチェーン終端剛体(形状=箱)のsize[1](PMX箱サイズ=半値なのでそのまま
    # 半高さ)×倍率。physicsGltf由来のサイズは衝突評価と同じ空間なので単位
    # 変換は不要(colliders_defと同じ流儀)。
    # hem_extend_scale=0.0(デフォルト)なら空辞書のままで既存挙動と完全に同じ。
    hem_extend = {}
    if hem_extend_scale > 0.0:
        for ch in chains:
            dyn = [p for p in ch.particles if p.rb in skirt_rbs and not p.kinematic]
            if not dyn:
                continue
            _tp = dyn[-1]
            if not (0 <= _tp.rb < len(rbs_names_all)):
                continue
            _rbdef = rbs_names_all[_tp.rb]
            if _rbdef.get("shape") != 1:   # 1=箱 以外は対象外
                continue
            _sz = _rbdef.get("size", ())
            if len(_sz) < 2 or _sz[1] <= 0:
                continue
            hem_extend[_tp.rb] = _sz[1] * hem_extend_scale
        if hem_extend:
            _vals = list(hem_extend.values())
            print("  [physics] hem_extend_scale=%.3f: %d skirt chain tail "
                  "segment(s) virtually extended for collision "
                  "(min=%.4f, max=%.4f, mean=%.4f)"
                  % (hem_extend_scale, len(hem_extend),
                     min(_vals), max(_vals), sum(_vals) / len(_vals)))
        else:
            print("  [physics] hem_extend_scale=%.3f: no eligible skirt chain "
                  "tail rigid body (shape=box) found; feature inactive"
                  % hem_extend_scale)

    # 横拘束(lateral)はスカートのリング(2次元クロス)専用。髪・ネクタイ・胸などの
    # チェーン型揺れ物は、モデルにストランド間ジョイントがあると非ツリー辺として
    # lateral 化され、ハード距離拘束で2次元網に固まり中間ボーンが偽平衡に固着する
    # (実測: IAの髪FLB2が頭姿勢に無関係に67°固着。lateral除去で静止1.4°/動的に解放)。
    # lateral は両端がスカート剛体のものだけ残す(髪等のチェーンは lateral=[] が本来の設計)。
    _lat_before = len(lateral)
    lateral = [(a, b, rl, sl, amin, amax, jrot) for (a, b, rl, sl, amin, amax, jrot) in lateral
               if a in skirt_rbs and b in skirt_rbs]
    if _lat_before != len(lateral):
        print("  [physics] dropped %d non-skirt lateral constraint(s) "
              "(hair/necktie/etc. are chains, not cloth)"
              % (_lat_before - len(lateral)))
    if lateral_slack_scale > 0.0 and lateral:
        _slacks = [sl * lateral_slack_scale for (_, _, _, sl, _amin, _amax, _jrot) in lateral]
        _nonzero = [s for s in _slacks if s > 0.0]
        if _nonzero:
            print("  [physics] lateral_slack_scale=%.3f: %d/%d lateral edge(s) "
                  "have nonzero play (min=%.4f, max=%.4f, mean=%.4f)"
                  % (lateral_slack_scale, len(_nonzero), len(lateral),
                     min(_nonzero), max(_nonzero), sum(_nonzero) / len(_nonzero)))

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

    # スカート(translation焼き対象)ボーンの回転出力は、既定では恒等(親と同じ
    # 向き)に固定する(skirt_bone_twist=False、既存挙動)。
    # 理由: これらのボーンは position(translation) で世界位置を厳密一致させている
    # ため、回転(向き)の値そのものはメッシュのツイスト表現以外に意味を持たない。
    # 一方で回転計算(q_from_to による rest 基準の目標方向)は、乱れたツリー構造
    # (リング横断等)を持つ揺れ物では目標方向と実方向がほぼ正反対になる瞬間があり、
    # 特異点で軸の選び方が不連続になって1フレームで大きく跳ねることがある(実測109°)。
    # ただしこの跳ねは simulate_step_cloth 内の q_slerp_toward(前フレームから
    # 最大MAX_ROT_DEG=60度までしか許容しない安全クランプ)で既に無効化された値が
    # seg_cloth/seg_otherに入っているため、skirt_bone_twist=Trueにすれば、その
    # 安全な回転値をそのままメッシュのツイスト表現に使える。実機フィードバック
    # (「区間の中点あたりに横方向へ押し出されたような段ができる」)は、回転
    # 情報を丸ごと捨てていたことでメッシュのツイストが表現できず、スキニングの
    # 補間が実際の傾きと食い違って起きていたと判断し追加。
    skirt_bone_set = set(p.bone for ch in chains for p in ch.particles if p.rb in skirt_rbs)

    # ======================================================================
    # 髪(その他揺れ物)と布(スカート等)でソルバの状態を完全に分離する。
    # 適応サブステップ(布のみ対象)を髪・胸などの物理から構造的に隔離するため。
    # 数学的な根拠: simulate_step_cloth 内の各処理(積分・縦/横拘束・衝突押し出し・
    # 回転出力)は、いずれもチェーン単位またはパーティクル単位で閉じており、他の
    # チェーン/パーティクルの結果には依存しない(resolve_collisions のみ全
    # パーティクルを1関数内でまとめて処理するが、これも1粒子ごとに独立な計算で
    # あり他粒子の結果を参照しない)。したがって「1つの状態に全チェーンをまとめて
    # 1回呼ぶ」のと「2つの状態に分けて2回呼ぶ」のとで、数値結果は変わらない。
    # 適応サブステップが不発動(N=1)のフレームでは、布側もこれまでと全く同じ
    # 引数で1回だけ simulate_step_cloth を呼ぶため、出力はこれまでと
    # ビット単位で一致する(adaptive_substep_threshold=None ならこの機構自体が
    # 無効なので、常にこの経路のみを通る)。
    chains_cloth = []
    chains_other = []
    for ch in chains:
        if any((not p.kinematic) and p.rb in skirt_rbs for p in ch.particles):
            chains_cloth.append(ch)
        else:
            chains_other.append(ch)
    _cloth_chain_ids = set(id(ch) for ch in chains_cloth)

    state_cloth = SpringState(chains_cloth, init_pos=init_pos)
    state_other = SpringState(chains_other)
    dt = 1.0 / fps

    # cloth_algorithm="rigid_chain"用の永続状態(各パーティクルの世界姿勢と
    # その前フレーム値。慣性はこの世界姿勢の差分で計算する。世界姿勢基準の
    # 理由はsimulate_step_rigid_chainのdocstring参照)。
    # フレームをまたいで保持する必要があるため、ここでstate_clothに載せておく
    # (SpringStateの通常の属性ではないが、Pythonのオブジェクトなので自由に追加できる)。
    # "points"(既定)では一切参照されない。
    if cloth_algorithm == "rigid_chain":
        state_cloth.world_q = {}
        state_cloth.prev_world_q = {}

    # cloth_algorithm="rigid_body"用のセットアップ(Stage 3: 並進+角速度を
    # 持つ本物の剛体チェーン)。ここで一度だけ剛体・拘束ペアを構築し、
    # 本ループでは毎フレームこれらを反復拘束解決するだけにする。
    if cloth_algorithm == "rigid_body":
        def _rb_basis_quat(y_axis, x_ref):
            """y_axis(単位ベクトル)をローカル+Yに、x_ref方向をできるだけ
            ローカル+Xに近づけた正規直交基底からクォータニオンを作る
            (グラム・シュミット直交化)。q_from_toだけだと軸まわりの捻り
            (ロール)が不定になり、横リングの接続点が実際の隣接パネル方向を
            向かずリングが崩れて1箇所に寄る不具合が実機フィードバックで
            見つかったため、横方向も明示的に固定する。
            """
            y = _norm(y_axis)
            x = _sub(x_ref, _scale(y, _dot(x_ref, y)))
            xl = _len(x)
            if xl < 1e-6:
                arbitrary = (1.0, 0.0, 0.0) if abs(y[0]) < 0.9 else (0.0, 0.0, 1.0)
                x = _sub(arbitrary, _scale(y, _dot(arbitrary, y)))
                xl = _len(x)
            x = _scale(x, 1.0 / xl) if xl > 1e-9 else (1.0, 0.0, 0.0)
            z = _cross(x, y)
            m = [[x[0], y[0], z[0], 0.0],
                 [x[1], y[1], z[1], 0.0],
                 [x[2], y[2], z[2], 0.0],
                 [0.0, 0.0, 0.0, 1.0]]
            _, q = mat_to_trs(m)
            return q

        # 各スカート剛体について、横リングでつながる隣接剛体を1つ探し、
        # bind pose での実際の方向をローカル+Xの参照として使う。
        _rb_lateral_ref = {}
        for (a_rb, b_rb, _rl, _sl, _amin, _amax, _jrot) in lateral:
            if a_rb in skirt_rbs and b_rb in skirt_rbs:
                _rb_lateral_ref.setdefault(a_rb, b_rb)
                _rb_lateral_ref.setdefault(b_rb, a_rb)

        rb_bodies = {}          # rb_index(動的パーティクル) -> RigidBodyXPBD
        rb_anchor_bodies = {}   # rb_index(kinematicアンカー) -> RigidBodyXPBD
        rb_vert_pairs = []      # (親body, 子body, 親がアンカーか)
        for ch in chains_cloth:
            plist = ch.particles
            anchor_p = plist[0]
            if anchor_p.rb not in rb_anchor_bodies:
                rb_anchor_bodies[anchor_p.rb] = RigidBodyXPBD(
                    anchor_p.rb, pos=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0),
                    mass=1.0, half_extents=(0.05, 0.05, 0.05), half_len=0.01,
                    kinematic=True)
            prev_body = rb_anchor_bodies[anchor_p.rb]
            prev_is_anchor = True
            for k in range(1, len(plist)):
                c = plist[k]; pa = plist[k-1]
                if c.rb in rb_bodies:
                    prev_body = rb_bodies[c.rb]; prev_is_anchor = False
                    continue
                rb_def = rbs_names_all[c.rb] if 0 <= c.rb < len(rbs_names_all) else {}
                mass = rb_def.get("mass", 1.0) or 1.0
                size = rb_def.get("size", (0.05, 0.05, 0.05))
                half_extents = tuple(max(s, 1e-4) for s in size) if size else (0.05, 0.05, 0.05)
                half_len = max(c.rest_len * 0.5, 1e-4)
                mid = tuple((x + y) * 0.5 for x, y in zip(pa.rest_pos, c.rest_pos)) \
                    if (pa.rest_pos and c.rest_pos) else (0.0, 0.0, 0.0)
                rest_dir_local = state_cloth.rest_dir.get(c.rb, (0.0, -1.0, 0.0))
                parent_dir = tuple(-x for x in rest_dir_local)
                nb_rb = _rb_lateral_ref.get(c.rb)
                nb_particle = parts.get(nb_rb) if nb_rb is not None else None
                if nb_particle is not None and nb_particle.rest_pos and c.rest_pos:
                    x_ref = _sub(nb_particle.rest_pos, c.rest_pos)
                else:
                    x_ref = (1.0, 0.0, 0.0)
                init_rot = _rb_basis_quat(parent_dir, x_ref)
                # PMXの実データが持つ移動減衰・回転減衰をそのまま使う(実測:
                # IAモデルのスカートは0.9/0.9。取得できない場合のみ0.1/0.3を保険で使う)。
                _lin_d = rb_def.get("linearDamping", 0.1)
                _ang_d = rb_def.get("angularDamping", 0.3)
                body = RigidBodyXPBD(c.rb, pos=mid, rot=init_rot, mass=mass,
                                     half_extents=half_extents, half_len=half_len,
                                     kinematic=False, ang_min=c.ang_min, ang_max=c.ang_max,
                                     joint_rot=c.joint_rot,
                                     lin_damping=_lin_d, ang_damping=_ang_d)
                rb_bodies[c.rb] = body
                rb_vert_pairs.append((prev_body, body, prev_is_anchor))
                prev_body = body; prev_is_anchor = False
        rb_lateral_pairs = [(a, b, rl, sl, amin, amax, jrot)
                           for (a, b, rl, sl, amin, amax, jrot) in lateral
                           if a in rb_bodies and b in rb_bodies]
        # 本家PMXの実データを確認したところ、スカート剛体は全て同じグループ
        # (group=2)で、非衝突マスクの自グループビットが0(=衝突する)になって
        # いた。つまり本家は「横リングで繋がった隣接パネルだけ」でなく、
        # 「全スカート剛体同士」が物理的に衝突するよう作者が意図的に設定して
        # いたと判明(実機フィードバックで発覚した、リングが崩れる問題の根本
        # 原因)。それに合わせ全ペアの自己衝突を行うが、36枚(630組)は純
        # Pythonには重いため、bind pose時点で十分離れている組は事前に除外
        # する(激しい動きでも触れ得るのは元々近いパネル同士が主、という
        # 前提の近似。既存の「今風にもっともらしく揺れればOK」という設計
        # 方針とも整合)。
        _rb_body_list = list(rb_bodies.values())
        _rb_bind_pos = {b.rb: b.pos for b in _rb_body_list}
        _rb_self_coll_max_dist = 0.0
        for b in _rb_body_list:
            _rb_self_coll_max_dist = max(_rb_self_coll_max_dist, b.half_w)
        _rb_self_coll_threshold = _rb_self_coll_max_dist * 6.0
        rb_self_collision_pairs = []
        for i in range(len(_rb_body_list)):
            for j in range(i + 1, len(_rb_body_list)):
                A, B = _rb_body_list[i], _rb_body_list[j]
                if _len(_sub(A.pos, B.pos)) <= _rb_self_coll_threshold:
                    rb_self_collision_pairs.append((A, B))
        rb_lin_damping = rigid_body_linear_damping
        rb_ang_damping = rigid_body_angular_damping
        rb_gravity_vec = _scale(_norm(gravity_dir), rigid_body_gravity)
        print("  [physics] cloth_algorithm=rigid_body: %d body(ies), %d vertical "
              "constraint(s), %d lateral constraint(s), %d self-collision pair(s) "
              "(of %d possible, bind-distance filtered)"
              % (len(rb_bodies), len(rb_vert_pairs), len(rb_lateral_pairs),
                 len(rb_self_collision_pairs),
                 len(_rb_body_list) * (len(_rb_body_list) - 1) // 2))

        def _rb_step(anchor_frame, colliders_frame=None, step_dt=None):
            """1フレーム(またはサブステップ)分の剛体チェーン解決。
            anchor_frame: {rb: (pos, rot)}。colliders_frame: そのフレームの
            コライダーリスト(resolve_collisionsと同じ形式)。Noneなら衝突判定
            なし(Stage 3序盤の挙動と同じ)。step_dt: このステップの時間刻み
            (省略時は1/fps。rigid_body_substeps>1のときは1/fpsをさらに割った
            値を渡す)。state_cloth.pos と戻り値のseg_rot(bone単位)を、既存の
            出力パイプライン(seg_rot_to_local・位置ベイク)がそのまま使える
            形で更新する。
            """
            sdt = step_dt if step_dt is not None else dt
            pred = {}
            for rb, ab in rb_anchor_bodies.items():
                a_pos, a_rot = anchor_frame.get(rb, (ab.pos, ab.rot))
                ab.pos, ab.rot = a_pos, a_rot
                pred[id(ab)] = (a_pos, a_rot)
            for b in rb_bodies.values():
                pred[id(b)] = _rb_integrate_predict(b, rb_gravity_vec, sdt,
                                                    rb_lin_damping, rb_ang_damping)

            for _ in range(rigid_body_iterations):
                for (A, B, A_is_anchor) in rb_vert_pairs:
                    pA, rA = pred[id(A)]; pB, rB = pred[id(B)]
                    pA, rA, pB, rB = _rb_solve_point_constraint(
                        pA, rA, A.inv_mass, A.inv_inertia_body, A.kinematic,
                        (0.0, 0.0, 0.0) if A_is_anchor else A.attach_child_local,
                        pB, rB, B.inv_mass, B.inv_inertia_body, B.kinematic,
                        B.attach_parent_local)
                    if B.ang_max is not None:
                        rA, rB = _rb_solve_angle_limit(
                            rA, rB, A.inv_inertia_body, B.inv_inertia_body,
                            A.kinematic, B.kinematic, B.ang_min, B.ang_max, B.joint_rot)
                    pred[id(A)] = (pA, rA); pred[id(B)] = (pB, rB)
                for (a_rb, b_rb, rl, sl, amin, amax, jrot) in rb_lateral_pairs:
                    A = rb_bodies[a_rb]; B = rb_bodies[b_rb]
                    pA, rA = pred[id(A)]; pB, rB = pred[id(B)]
                    pA, rA, pB, rB = _rb_solve_distance_constraint(
                        pA, rA, A.inv_mass, A.inv_inertia_body, A.kinematic, (A.half_w, 0.0, 0.0),
                        pB, rB, B.inv_mass, B.inv_inertia_body, B.kinematic, (-B.half_w, 0.0, 0.0),
                        rl, sl * lateral_slack_scale)
                    if amin is not None:
                        rA, rB = _rb_solve_angle_limit(
                            rA, rB, A.inv_inertia_body, B.inv_inertia_body,
                            A.kinematic, B.kinematic, amin, amax, jrot)
                    pred[id(A)] = (pA, rA); pred[id(B)] = (pB, rB)
                # 本家のPMXグループ設定(スカート同士は自グループ衝突が有効)を
                # 再現する全ペア自己衝突。PMXの実測減衰値(例: 0.9/0.9)を使う
                # 場合、反復の外で1回だけだと収束不足でリングが崩れると実機
                # フィードバックで発覚したため、精度を優先し反復の中に戻した
                # (速度は犠牲になるが、モデルの実測値を歪めるよりはこちらを
                # 優先すべきとの判断)。
                for (A, B) in rb_self_collision_pairs:
                    pA, rA = pred[id(A)]; pB, rB = pred[id(B)]
                    pA, pB = _rb_solve_min_separation(
                        pA, A.inv_mass, A.kinematic, pB, B.inv_mass, B.kinematic,
                        (A.half_w + B.half_w) * rigid_body_self_collision_scale)
                    pred[id(A)] = (pA, rA); pred[id(B)] = (pB, rB)
                if colliders_frame:
                    for rb, b in rb_bodies.items():
                        pB, rB = pred[id(b)]
                        pB = _rb_solve_collision_pos(pB, colliders_frame, collision_margin)
                        pred[id(b)] = (pB, rB)

            for b in rb_bodies.values():
                pos_new, rot_new = pred[id(b)]
                new_vel = _scale(_sub(pos_new, b.pos), 1.0 / sdt)
                dq = q_mul(rot_new, q_conj(b.rot))
                if dq[3] < 0:
                    dq = tuple(-c for c in dq)
                al = _len(dq[:3])
                new_omega = _scale(dq[:3], 2.0 / sdt) if al > 1e-9 else (0.0, 0.0, 0.0)
                # 本家Bullet(btRigidBody::integrateVelocities)にも角速度の上限
                # クランプ(MAX_ANGVEL=π/2)があり、1ステップでの過大な暗黙速度
                # による不安定化を防いでいる。この実装には無かったため追加。
                # 特にモーション冒頭やpriming pass直後のように、アンカーが
                # 1フレームで大きく飛ぶ場面で、そこから生じる過大な速度が
                # 次フレーム以降も伝播し続けて「一度崩れると戻らない」不安定化
                # を招く一因と考えられる(実機/文献調査で判明: MMD本家にも
                # モーション読み込み時の急変を物理エンジンが「衝撃」と捉え、
                # 一部モデルは自己持続的に崩れたまま戻らないという既知の未解決
                # 問題があり、他エンジンの変更履歴にも「生成直後にポーズが
                # 急変した場合はジャンプさせない」という同種の対策が見られる)。
                vlen = _len(new_vel)
                if vlen > rigid_body_max_linear_speed:
                    new_vel = _scale(new_vel, rigid_body_max_linear_speed / vlen)
                olen = _len(new_omega)
                if olen > rigid_body_max_angular_speed:
                    new_omega = _scale(new_omega, rigid_body_max_angular_speed / olen)
                b.vel = new_vel
                b.omega = new_omega
                b.pos, b.rot = pos_new, rot_new

            seg = {}
            for rb, body in rb_bodies.items():
                child_world = _add(body.pos, q_rotate_vec(body.rot, (0.0, -body.half_len, 0.0)))
                state_cloth.pos[rb] = child_world
                rdir = state_cloth.rest_dir.get(rb)
                if rdir is not None:
                    parent_world = _add(body.pos, q_rotate_vec(body.rot, (0.0, body.half_len, 0.0)))
                    cur_dir = _norm(_sub(child_world, parent_world))
                    seg[rb] = q_from_to(rdir, cur_dir)
            return seg

        def _rb_step_substeps(anchor_prev, anchor_cur, colliders_frame, n_sub):
            """1フレームをn_sub個のサブステップに分割し、アンカーの位置・姿勢を
            区間内で補間しながら_rb_stepを複数回呼ぶ。VMDViewerが60FPS(=この
            ツールの既定30FPSの倍の時間解像度)で動いているとの実機情報を受け、
            1ステップあたりの誤差を小さくして反復拘束解決の収束を助ける狙い。
            anchor_prev: 前フレームのアンカー({rb:(pos,rot)}、フレーム0では
            anchor_curと同じものを渡せば補間なしで従来と同じ挙動になる)。
            戻り値: 最後のサブステップのseg(出力用、中間は使わない)。
            """
            if n_sub <= 1:
                return _rb_step(anchor_cur, colliders_frame, step_dt=dt)
            seg = None
            sdt = dt / n_sub
            for i in range(1, n_sub + 1):
                t = i / n_sub
                a_interp = _lerp_anchor(anchor_prev, anchor_cur, t)
                seg = _rb_step(a_interp, colliders_frame, step_dt=sdt)
            return seg

    def _lerp(a, b, t):
        return _add(a, _scale(_sub(b, a), t))

    def _slerp_q(a, b, t):
        d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
        if d < 0.0:
            b = (-b[0], -b[1], -b[2], -b[3]); d = -d
        d = max(-1.0, min(1.0, d))
        theta0 = math.acos(d)
        if theta0 < 1e-9:
            return a
        theta = theta0 * t
        s0 = math.sin(theta0 - theta) / math.sin(theta0)
        s1 = math.sin(theta) / math.sin(theta0)
        return (a[0]*s0+b[0]*s1, a[1]*s0+b[1]*s1,
               a[2]*s0+b[2]*s1, a[3]*s0+b[3]*s1)

    def _lerp_anchor(a_prev, a_cur, t):
        out = {}
        for rb, (p, q) in a_cur.items():
            pp, pq = a_prev.get(rb, (p, q))
            out[rb] = (_lerp(pp, p, t), _slerp_q(pq, q, t))
        return out

    def _set_anchors(state, a):
        for rb, (wp, wr) in a.items():
            if rb in state.part:
                state.set_anchor(rb, wp, wr)

    def _resync_rigid_chain_after_lateral(seg_cloth_dict):
        """横リング拘束(_apply_lateral_constraint)がstate_cloth.posを直接
        書き換えた後、cloth_algorithm="rigid_chain"のworld_q/seg_rotを
        「実際に置かれた位置」に合わせて再計算する。これをしないと、次
        フレームの慣性(world_qの差分)が横方向の補正を反映せず、姿勢と
        位置が食い違ったまま蓄積してしまう。simulate_step_rigid_chain
        と同じトップダウン連鎖規則(q_par)で、全スカートパーティクルを
        位置から素直に再計算するだけ(角度制限などの再適用はしない、
        横方向の補正量は通常小さいため次フレームの積分で自然に収束する)。
        warmupループ・本ループの両方から使うため、ここ(warmupより前)で定義する。
        """
        pos = state_cloth.pos
        for ch in state_cloth.chains:
            plist = ch.particles
            q_par = state_cloth._anchor_rot.get(plist[0].rb, (0.0, 0.0, 0.0, 1.0)) \
                if plist[0].kinematic else (0.0, 0.0, 0.0, 1.0)
            for k in range(1, len(plist)):
                c = plist[k]; pa = plist[k-1]
                rdir = state_cloth.rest_dir.get(c.rb)
                if rdir is None:
                    q_par = seg_cloth_dict.get(c.rb, q_par)
                    continue
                tgt_dir = q_rotate_vec(q_par, rdir)
                cur_dir = _norm(_sub(pos[c.rb], pos[pa.rb]))
                new_wq = q_mul(q_from_to(tgt_dir, cur_dir), q_par)
                if c.inv_mass > 0:
                    state_cloth.world_q[c.rb] = new_wq
                seg_cloth_dict[c.rb] = new_wq
                q_par = seg_cloth_dict[c.rb]

    warmup = 20   # 修正1のinit_posシードで十分。warmup増は髪の揺れを損なうため20維持
    # cloth_algorithm="rigid_body"は、上のwarmupとは独立したフレーム数を使う。
    # bind poseから重力で自然に垂れるまでの「助走」がここに20フレーム
    # (0.67秒)しか無いと、遷移が急に見えるという実機フィードバックを受けて
    # 分離した(点ベース/rigid_chainのwarmupは既存のまま変更しない)。
    warmup_rigid_body = max(warmup, int(round(rigid_body_warmup_seconds * fps)))
    a0 = anchor_fn(0)
    _warmup_this_algo = warmup_rigid_body if cloth_algorithm == "rigid_body" else warmup
    for _ in range(_warmup_this_algo):
        _set_anchors(state_other, a0)
        _set_anchors(state_cloth, a0)
        simulate_step_cloth(state_other, gravity_dir, dt, drag_force, stiffness_force,
                            [], gravity_power=gravity_power, iterations=6,
                            colliders=colliders_at(0), body_axis=body_axis_at(0),
                            radial_rbs=skirt_rbs, margin=collision_margin,
                            exclude_rb=exclude_rb, skip_angle_clamp_rbs=skirt_rbs,
                            allowed_collider_rbi=allowed_collider_rbi,
                            hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                            collision_bounce=collision_bounce,
                            extra_margin_by_rb=rb_size_extra,
                            margin_cap_by_pair=_pair_cap,
                            collision_velocity_damp=collision_velocity_damp,
                            push_report=(_push_report if physics_report else None),
                            push_preserve_length=push_preserve_length,
                            skip_embedded_parent=skip_embedded_parent,
                            lateral_slack_scale=lateral_slack_scale,
                            vertical_spread_scale=vertical_spread_scale,
                            vertical_neighbors=vertical_neighbors)
        if cloth_algorithm == "rigid_chain":
            _warmup_seg = simulate_step_rigid_chain(
                state_cloth.chains, state_cloth.rest_dir, {}, state_cloth.part,
                state_cloth.pos, state_cloth._anchor_rot,
                state_cloth.world_q, state_cloth.prev_world_q,
                gravity_dir, dt, drag_force, rigid_chain_stiffness_force,
                gravity_power=rigid_chain_gravity_power)
            _apply_lateral_constraint(state_cloth.pos, state_cloth.part,
                                      lateral, lateral_slack_scale)
            _resync_rigid_chain_after_lateral(_warmup_seg)
        elif cloth_algorithm == "rigid_body":
            _rb_step_substeps(a0, a0, colliders_at(0), rigid_body_substeps)
        else:
            simulate_step_cloth(state_cloth, gravity_dir, dt, drag_force, stiffness_force,
                                lateral, gravity_power=gravity_power, iterations=6,
                                colliders=colliders_at(0), body_axis=body_axis_at(0),
                                radial_rbs=skirt_rbs, margin=collision_margin,
                                exclude_rb=exclude_rb, skip_angle_clamp_rbs=skirt_rbs,
                                allowed_collider_rbi=allowed_collider_rbi,
                                hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                                collision_bounce=collision_bounce,
                                extra_margin_by_rb=rb_size_extra,
                            margin_cap_by_pair=_pair_cap,
                            collision_velocity_damp=collision_velocity_damp,
                            push_report=(_push_report if physics_report else None),
                            push_preserve_length=push_preserve_length,
                            skip_embedded_parent=skip_embedded_parent,
                                lateral_slack_scale=lateral_slack_scale,
                                vertical_spread_scale=vertical_spread_scale,
                                vertical_neighbors=vertical_neighbors,
                                segment_aware_collision=segment_aware_collision,
                                hem_extend=hem_extend)

    # cloth_algorithm="rigid_body"専用: 全モーションを通しで一度「助走」させて
    # から、frame0から本番の記録をやり直す2パス方式。実機フィードバックで
    # 判明した本家VMDビューアの仕様(初回再生はフレーム0の物理がぐちゃぐちゃ
    # だが、一度最後まで再生してフレーム0に戻ると、体のボーンはframe0の
    # キーフレームに戻る一方、物理(スカート・髪)は最後に落ち着いた状態を
    # 引き継ぐため2回目から狙った動きになる)を模す。1パス目は出力を記録
    # しない(体の姿勢だけ動かし、rb_bodiesの状態を実際の全モーション分
    # 動かしておく)。
    if cloth_algorithm == "rigid_body" and rigid_body_prime_full_motion:
        print("  [physics] rigid_body: priming pass over the full motion "
              "(%d frames) before the recorded pass, matching the real "
              "MMD viewer's 'first playthrough settles physics' behavior"
              % num_frames)
        prev_a_prime = anchor_fn(0)
        for f in range(num_frames):
            a_prime = anchor_fn(f)
            _rb_step_substeps(prev_a_prime, a_prime, colliders_at(f), rigid_body_substeps)
            prev_a_prime = a_prime

    # ------------------------------------------------------------------
    # 適応サブステップ(布のみ対象): フレーム間の「布アンカー変位＋近傍コライダー
    # 変位」の合計(三角不等式による安全側の上限見積もり。フェーズAの計測と
    # 同じ考え方)が、最寄りコライダーの半径 × adaptive_substep_threshold を
    # 超えたフレームだけ、布ソルバを N 分割して呼ぶ。
    # ------------------------------------------------------------------
    _substep_enabled = adaptive_substep_threshold is not None
    chain_reach = {}
    _substep_collider_defs = colliders_def
    _substep_slack = 0.3 + max(collision_margin, 0.0)
    if _substep_enabled:
        for ch in chains_cloth:
            p0 = ch.particles[0]
            if p0.kinematic:
                reach = sum(p.rest_len for p in ch.particles[1:])
                chain_reach[p0.rb] = max(chain_reach.get(p0.rb, 0.0), reach)
        if adaptive_substep_collider_names is not None:
            _allow = set(adaptive_substep_collider_names)
            _substep_collider_defs = [c for c in colliders_def
                                      if rbs_names_all[c[-1]]["name"] in _allow]
        print("  [physics] adaptive substep: threshold=%.3f (x collider radius), "
              "max_N=%d, %d collider(s) considered"
              % (adaptive_substep_threshold, adaptive_substep_max_n,
                 len(_substep_collider_defs)))

    def _collider_centers(cols, allowed_rbi):
        out = {}
        for col in cols:
            if col[0] == "sphere":
                _, c, rad, _grp, _msk, rbi = col
            else:
                _, p0, p1, rad, _grp, _msk, rbi = col
                c = ((p0[0]+p1[0])*0.5, (p0[1]+p1[1])*0.5, (p0[2]+p1[2])*0.5)
            if allowed_rbi is not None and rbi not in allowed_rbi:
                continue
            out[rbi] = (c, rad)
        return out

    _substep_allowed_rbi = (set(c[-1] for c in _substep_collider_defs)
                            if adaptive_substep_collider_names is not None else None)

    def _compute_substep_n(anchor_prev, anchor_cur, collider_prev, collider_cur):
        best_ratio = 0.0
        for rb, reach in chain_reach.items():
            p, _q = anchor_cur.get(rb, (None, None))
            if p is None:
                continue
            pp, _pq = anchor_prev.get(rb, (p, _q))
            a_disp = _len(_sub(p, pp))
            local_max = 0.0
            local_rad = 0.0
            for rbi, (c_now, rad) in collider_cur.items():
                dist = _len(_sub(p, c_now))
                if dist > reach + rad + _substep_slack:
                    continue
                c_prev, _ = collider_prev.get(rbi, (c_now, rad))
                c_disp = _len(_sub(c_now, c_prev))
                if c_disp > local_max:
                    local_max = c_disp
                    local_rad = rad
            if local_rad <= 1e-6:
                continue
            ratio = (a_disp + local_max) / local_rad
            if ratio > best_ratio:
                best_ratio = ratio
        if best_ratio <= 0.0:
            return 1
        n = math.ceil(best_ratio / adaptive_substep_threshold)
        return max(1, min(adaptive_substep_max_n, n))

    def _lerp_colliders(cols_prev, cols_cur, t):
        prev_by_rbi = {c[-1]: c for c in cols_prev}
        out = []
        for col in cols_cur:
            rbi = col[-1]
            pcol = prev_by_rbi.get(rbi, col)
            if col[0] == "sphere":
                _, c, rad, grp, msk, _rbi = col
                _, pc, _prad, _pgrp, _pmsk, _prbi = pcol
                out.append(("sphere", _lerp(pc, c, t), rad, grp, msk, rbi))
            else:
                _, p0, p1, rad, grp, msk, _rbi = col
                _, pp0, pp1, _prad, _pgrp, _pmsk, _prbi = pcol
                out.append(("capsule", _lerp(pp0, p0, t), _lerp(pp1, p1, t),
                           rad, grp, msk, rbi))
        return out

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

    # ------------------------------------------------------------------
    # 中間パーティクル補正(布のみ): 隣接するチェーン同士を結ぶ横リングの辺
    # (lateral、同じ高さの輪を構成する辺)の途中がコライダーに食い込んで
    # いたら、その両端の実ボーン位置を少しだけ押し出す。縦方向(チェーン内の
    # 親子、腰→裾へ垂れる方向)のセグメントは対象にしない — 根元に近い縦
    # セグメントを押すと、その1本のズレがチェーンを伝って裾側全体を広げて
    # しまう(傘化)ため。横リングだけを対象にすることで、この縦方向の伝播を
    # 避けつつ、輪の途中がコライダーにめり込む見た目を緩和する。
    # 押し出し方向はコライダー表面から最短距離で離れる向き(制限なし)。
    # 検証の結果、貫入の深さ・面積は6〜7割程度緩和できるが、「貫入が発生する
    # フレーム数」自体はほぼ変わらない(浅く広く残る)緩和策。
    # ------------------------------------------------------------------
    _midpoint_enabled = midpoint_correction
    _drape_probe_on = bool(drape_probe and _drape_active)
    _midpoint_allowed_rbi = None
    if _midpoint_enabled and midpoint_correction_collider_names is not None:
        _allow_mp = set(midpoint_correction_collider_names)
        _midpoint_allowed_rbi = set(
            c[-1] for c in colliders_def
            if rbs_names_all[c[-1]]["name"] in _allow_mp)

    def _midpoint_push(m, cols, extra_margin=0.0):
        best_depth = 0.0
        best_dir = None
        for col in cols:
            if _midpoint_allowed_rbi is not None and col[-1] not in _midpoint_allowed_rbi:
                continue
            if col[0] == "sphere":
                C, rad = col[1], col[2]
            else:
                C = _closest_on_segment(m, col[1], col[2]); rad = col[3]
            dvec = _sub(m, C)
            d = _len(dvec)
            R = rad + midpoint_correction_margin + extra_margin
            if d < R:
                depth = R - d
                if depth > best_depth:
                    best_depth = depth
                    best_dir = _norm(dvec) if d > 1e-9 else (0.0, 1.0, 0.0)
        if best_dir is None:
            return None
        return _scale(best_dir, best_depth)

    def _apply_midpoint_correction(a_full, cols, axis=None):
        n_samples = max(1, midpoint_correction_samples)
        ts = [si / (n_samples + 1) for si in range(1, n_samples + 1)]
        for _ in range(midpoint_correction_iters):
            for rb_a, rb_b, rest_len, _lat_slack, _amin, _amax, _jrot in lateral:
                pa = state_cloth.part.get(rb_a)
                pb = state_cloth.part.get(rb_b)
                if pa is None or pb is None:
                    continue
                # hem_extra_margin をボーン本体(resolve_collisions)と同じ
                # 「根元0→裾で全量」の深さ比例で中間パーティクルにも一貫適用する。
                # エッジの深さは両端パーティクルの hem_weight の平均。
                # hem_extra_margin=0 なら加算0で従来挙動と完全に同じ。
                _hw = (hem_weight.get(rb_a, 0.0) + hem_weight.get(rb_b, 0.0)) * 0.5
                # rb_size_extra(剛体実サイズ由来)も同様に両端の平均をボーン本体と
                # 一貫適用する。rb_size_margin_scale=0.0(既定)なら常に空辞書で加算0。
                _rse = (rb_size_extra.get(rb_a, 0.0) + rb_size_extra.get(rb_b, 0.0)) * 0.5
                # drape_probe: 深さを「マージンの上乗せ」ではなく「サンプル点を
                # 内側へ寄せる」形で表現する。中間パーティクルが実際のドレープの
                # 深い位置に置かれるので、押し出しの向きも布の内側基準になる。
                # 位置と量の二重計上を避けるため、_rse はマージンから外す。
                if _drape_probe_on:
                    _edge_extra = hem_extra_margin * _hw
                    _probe_in = _rse
                else:
                    _edge_extra = hem_extra_margin * _hw + _rse
                    _probe_in = 0.0
                for t in ts:
                    if pa.kinematic:
                        if rb_a not in a_full:
                            continue
                        pos_a = a_full[rb_a][0]
                    else:
                        pos_a = state_cloth.pos[rb_a]
                    if pb.kinematic:
                        if rb_b not in a_full:
                            continue
                        pos_b = a_full[rb_b][0]
                    else:
                        pos_b = state_cloth.pos[rb_b]
                    m = _lerp(pos_a, pos_b, t)
                    if _probe_in > 0.0 and axis is not None:
                        # 体の軸(腰ボーン)から見た水平の外向きを求め、その逆へ寄せる。
                        # 体が傾いてもアンカーが動くので追従する。
                        _bc = axis[0]
                        _hx, _hz = m[0] - _bc[0], m[2] - _bc[2]
                        _hl = math.hypot(_hx, _hz)
                        if _hl > 1e-6:
                            m = (m[0] - _hx / _hl * _probe_in, m[1],
                                 m[2] - _hz / _hl * _probe_in)
                    corr = _midpoint_push(m, cols, _edge_extra)
                    if corr is None:
                        continue
                    if pa.kinematic and pb.kinematic:
                        continue
                    elif pa.kinematic:
                        state_cloth.pos[rb_b] = _add(pos_b, corr)
                    elif pb.kinematic:
                        state_cloth.pos[rb_a] = _add(pos_a, corr)
                    else:
                        state_cloth.pos[rb_a] = _add(pos_a, _scale(corr, 1.0 - t))
                        state_cloth.pos[rb_b] = _add(pos_b, _scale(corr, t))
                    base_a = pos_a if pa.kinematic else state_cloth.pos[rb_a]
                    d2 = _sub(state_cloth.pos[rb_b] if not pb.kinematic else pos_b, base_a)
                    dl = _len(d2)
                    if dl > 1e-9 and not pb.kinematic:
                        state_cloth.pos[rb_b] = _add(base_a, _scale(d2, rest_len / dl))
            # 横リングの押し出しで動かした粒子は、縦方向のチェーン拘束
            # (親子間の長さ=rest_len)から見ると長さが崩れている。そのままだと
            # チェーンが伸び縮みし、見た目が破綻する(傘化含む)ため、
            # ルートから順に縦方向の長さを rest_len へ戻す「念押し」を行う。
            # これにより、横リングの押し出しが「セグメントの向きを横に曲げる」
            # 効果として残り、「セグメントを伸ばす」効果は打ち消される。
            for ch in chains_cloth:
                plist = ch.particles
                for k in range(1, len(plist)):
                    c = plist[k]; pa2 = plist[k-1]
                    if c.rb not in skirt_rbs:
                        continue
                    if pa2.kinematic:
                        if pa2.rb not in a_full:
                            continue
                        base = a_full[pa2.rb][0]
                    else:
                        base = state_cloth.pos[pa2.rb]
                    d2 = _sub(state_cloth.pos[c.rb], base)
                    dl = _len(d2)
                    if dl > 1e-9:
                        state_cloth.pos[c.rb] = _add(base, _scale(d2, c.rest_len / dl))

    keys = {}    # bone -> [(f, quat_local)]
    tkeys = {}   # bone -> [(f, (x,y,z) local translation)]  スカート等のみ
    _ndyn = sum(1 for ch in chains for pp in ch.particles if not pp.kinematic)
    print("  [physics] baking %d cloth bones over %d frames..."
          % (_ndyn, num_frames))
    if _midpoint_enabled:
        print("  [physics] midpoint correction: iters=%d, margin=%.4f"
              % (midpoint_correction_iters, midpoint_correction_margin))

    prev_anchor_full = a0            # frame -1相当 = warmupの最終姿勢(frame0と同じ)
    # --- 計測: スカートの各リングが静止時の半径からどれだけ開いたかを毎フレーム記録する。
    # 「接触で開く」のか「開いたまま戻らない」のかを、目視でなく数字で切り分けるため。
    _rep_rings, _rep_rest, _rep_series = {}, {}, {}
    _rep_part = {}
    _rep_axis0 = body_axis_at(0) if physics_report else None
    if _rep_axis0 and skirt_rbs:
        _bc0 = _rep_axis0[0]
        for ch in chains:
            for k, p in enumerate(ch.particles):
                if p.rb not in skirt_rbs or p.kinematic:
                    continue
                if not (0 <= p.bone < len(nodes)):
                    continue
                _rp = bone_world_pos(p.bone, 0)
                _rep_rest[p.rb] = math.hypot(_rp[0] - _bc0[0], _rp[2] - _bc0[2])
                _rep_part[p.rb] = p
                _rep_rings.setdefault(k, []).append(p.rb)
        for _k in _rep_rings:
            _rep_series[_k] = []
    _rep_near = {}
    # レポート用: メッシュ実測のドレープ深さ(glTF単位)。ボーンが当たっていなくても
    # 「谷」がこの深さぶん内側にあるので、クリアランスがこれを下回るとメッシュは
    # 見た目に貫通する。drape_depth_scale とは独立に、計測のためだけに使う。
    _rep_drape = {}
    if physics_report and drape_depth_by_bone:
        for ch in chains:
            for p in ch.particles:
                if p.kinematic or p.rb not in skirt_rbs:
                    continue
                _dv = drape_depth_by_bone.get(p.bone)
                if _dv and _dv > 0.0:
                    _rep_drape[p.rb] = _dv * scale
    _rep_pen = {}
    _push_report = {}

    prev_colliders_full = colliders_at(0)
    # 適応サブステップの発動回数を集計するカウンタ。以前はn_sub>1のフレームで
    # 毎回1行printしていたが、発動率が高いモーションでは数千行に達し
    # (実測: ぽんぷで7000フレーム中2820フレーム)、ログの可読性を落とすだけで
    # なくGUI側のテキスト挿入負荷にもなっていた。実際に発動したかどうかの
    # 情報自体はベイク結果に一切影響しない診断用の出力なので、定期的な進捗
    # 表示にまとめて件数だけ載せる形に変更(この変更はログ出力のみで、物理
    # 計算・ベイク結果には無関係)。
    _substep_fire_total = 0
    _substep_fire_since_checkpoint = 0

    for f in range(num_frames):
        if f % 500 == 0 and f > 0:
            print("  [physics]   frame %d/%d (substep fired %d since last)"
                  % (f, num_frames, _substep_fire_since_checkpoint))
            _substep_fire_since_checkpoint = 0
        a = anchor_fn(f)
        cols_f = colliders_at(f)
        body_axis_f = body_axis_at(f)

        n_sub = 1
        if _substep_enabled:
            n_sub = _compute_substep_n(
                prev_anchor_full, a,
                _collider_centers(prev_colliders_full, _substep_allowed_rbi),
                _collider_centers(cols_f, _substep_allowed_rbi))
            if n_sub > 1:
                _substep_fire_total += 1
                _substep_fire_since_checkpoint += 1

        # 髪等(state_other)は常にN=1・無改造
        _set_anchors(state_other, a)
        seg_other = simulate_step_cloth(state_other, gravity_dir, dt, drag_force,
                                        stiffness_force, [],
                                        gravity_power=gravity_power, iterations=6,
                                        colliders=cols_f, body_axis=body_axis_f,
                                        radial_rbs=skirt_rbs, margin=collision_margin,
                                        exclude_rb=exclude_rb, skip_angle_clamp_rbs=skirt_rbs,
                                        allowed_collider_rbi=allowed_collider_rbi,
                                        hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                                        collision_bounce=collision_bounce,
                                        extra_margin_by_rb=rb_size_extra,
                            margin_cap_by_pair=_pair_cap,
                            collision_velocity_damp=collision_velocity_damp,
                            push_report=(_push_report if physics_report else None),
                            push_preserve_length=push_preserve_length,
                            skip_embedded_parent=skip_embedded_parent,
                                        lateral_slack_scale=lateral_slack_scale,
                                        vertical_spread_scale=vertical_spread_scale,
                                        vertical_neighbors=vertical_neighbors)

        if cloth_algorithm == "rigid_chain":
            # 【実験的】板(剛体チェーン)方式。適応サブステップ・中間パーティクル
            # 補正・コライダー衝突は現時点で未対応(縦の回転駆動＋横リングのみ、
            # Stage 1/2のスコープ)。n_subは"points"専用の機構なので参照しない。
            _set_anchors(state_cloth, a)
            seg_cloth = simulate_step_rigid_chain(
                state_cloth.chains, state_cloth.rest_dir, {}, state_cloth.part,
                state_cloth.pos, state_cloth._anchor_rot,
                state_cloth.world_q, state_cloth.prev_world_q,
                gravity_dir, dt, drag_force, rigid_chain_stiffness_force,
                gravity_power=rigid_chain_gravity_power)
            _apply_lateral_constraint(state_cloth.pos, state_cloth.part,
                                      lateral, lateral_slack_scale)
            _resync_rigid_chain_after_lateral(seg_cloth)
        elif cloth_algorithm == "rigid_body":
            # 【実験的 Stage 3】並進+角速度を持つ本物の剛体チェーン。適応
            # サブステップ・中間パーティクル補正は現時点で未対応(縦の点拘束+
            # 角度制限＋横リング距離拘束＋簡易衝突押し出しのみ)。
            # _set_anchorsはseg_rot_to_local用にstate_cloth._anchor_rotを
            # 更新するために必要(剛体自体のアンカー位置はrb_anchor_bodies側で
            # 個別に管理している)。
            _set_anchors(state_cloth, a)
            seg_cloth = _rb_step_substeps(prev_anchor_full, a, cols_f, rigid_body_substeps)
        elif n_sub <= 1:
            # 不発動フレーム: これまでと完全に同じ1回呼び出し(ビット単位で一致)。
            _set_anchors(state_cloth, a)
            seg_cloth = simulate_step_cloth(state_cloth, gravity_dir, dt, drag_force,
                                            stiffness_force, lateral,
                                            gravity_power=gravity_power, iterations=6,
                                            colliders=cols_f, body_axis=body_axis_f,
                                            radial_rbs=skirt_rbs, margin=collision_margin,
                                            exclude_rb=exclude_rb, skip_angle_clamp_rbs=skirt_rbs,
                                            allowed_collider_rbi=allowed_collider_rbi,
                                            hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                                            collision_bounce=collision_bounce,
                                            extra_margin_by_rb=rb_size_extra,
                            margin_cap_by_pair=_pair_cap,
                            collision_velocity_damp=collision_velocity_damp,
                            push_report=(_push_report if physics_report else None),
                            push_preserve_length=push_preserve_length,
                            skip_embedded_parent=skip_embedded_parent,
                                            lateral_slack_scale=lateral_slack_scale,
                                            vertical_spread_scale=vertical_spread_scale,
                                            vertical_neighbors=vertical_neighbors,
                                            segment_aware_collision=segment_aware_collision,
                                            hem_extend=hem_extend)
            if _midpoint_enabled:
                # N=1(実質1サブステップ)のフレームでは、通常呼び出し直後に1回だけ補正。
                _apply_midpoint_correction(a, cols_f, body_axis_at(f))
        else:
            # 発動フレーム: アンカー・コライダーをN分割補間しつつN回呼ぶ。
            # gravity_power は 1/N でスケーリングし、N回分の積み重ねが1回分と
            # 釣り合うようにする(dtは変更しない)。
            # drag は「減衰率」で、積分では速度に (1 - drag) が掛かる(保持率が
            # 1-drag)。N回の積み重ねで保持率が (1-drag) に戻るように、
            # サブステップごとの保持率を (1-drag)^(1/N) にする。
            # 旧実装 drag_force**(1/N) は「drag=保持率」という誤ったモデルの
            # 合成則で、drag=0.85・N=2なら保持率が意図の15%に対し0.6%、N=4で
            # ほぼ0となり、発動フレーム(=一番勢いが欲しい速い動きのフレーム)
            # だけ慣性が毎回消える過減衰を招いていた(実機フィードバック
            # 「サブステップを有効にすると減衰が早くなる」の原因)。
            grav_sub = gravity_power / n_sub
            drag_sub = 1.0 - (1.0 - drag_force) ** (1.0 / n_sub)
            # stiffness は積分内で毎呼び出し stiffness_force*dt の変位を rest 方向へ
            # 加算するため、無スケールだとN回でフレームあたりN倍の引き戻しになる。
            # gravity_power と同格の 1/N でスケーリングし、N回分の合計が1回分と
            # 釣り合うようにする(拘束反復が間に挟まるため厳密等価にはならないが、
            # N倍の系統的な偏りは除去される)。
            stiff_sub = stiffness_force / n_sub
            for si in range(1, n_sub + 1):
                t = si / n_sub
                a_sub = _lerp_anchor(prev_anchor_full, a, t)
                cols_sub = _lerp_colliders(prev_colliders_full, cols_f, t)
                _set_anchors(state_cloth, a_sub)
                seg_cloth = simulate_step_cloth(state_cloth, gravity_dir, dt, drag_sub,
                                                stiff_sub, lateral,
                                                gravity_power=grav_sub, iterations=6,
                                                colliders=cols_sub, body_axis=body_axis_f,
                                                radial_rbs=skirt_rbs, margin=collision_margin,
                                                exclude_rb=exclude_rb, skip_angle_clamp_rbs=skirt_rbs,
                                                allowed_collider_rbi=allowed_collider_rbi,
                                                hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                                                collision_bounce=collision_bounce,
                                                extra_margin_by_rb=rb_size_extra,
                            margin_cap_by_pair=_pair_cap,
                            collision_velocity_damp=collision_velocity_damp,
                            push_report=(_push_report if physics_report else None),
                            push_preserve_length=push_preserve_length,
                            skip_embedded_parent=skip_embedded_parent,
                                                lateral_slack_scale=lateral_slack_scale,
                                                vertical_spread_scale=vertical_spread_scale,
                                                vertical_neighbors=vertical_neighbors,
                                                segment_aware_collision=segment_aware_collision,
                                                hem_extend=hem_extend)
                if _midpoint_enabled:
                    # 「逆順」= サブステップ1回ごとに毎回補正をかける。フレーム
                    # 終わりに1回だけ補正するより、検証の結果わずかに有利
                    # (貫入面積・貫入フレーム数の両方でごくわずかに改善)だった。
                    _apply_midpoint_correction(a_sub, cols_sub, body_axis_at(f))

        loc = {}
        loc.update(seg_rot_to_local(state_other, seg_other))
        loc.update(seg_rot_to_local(state_cloth, seg_cloth))
        if not skirt_bone_twist:
            for bone in skirt_bone_set:
                if bone in loc:
                    loc[bone] = (0.0, 0.0, 0.0, 1.0)
        for bone, q in loc.items():
            keys.setdefault(bone, []).append((f, q))
        # 20フレームごとに「各スカートパーティクルが今どのコライダーに一番近いか」を集計。
        # 押し出しが効き続けているなら、その相手の表面距離がマージン相当で張り付く。
        if _rep_rest and (f % 20 == 0):
            for _k, _rbl in _rep_rings.items():
                for _rb in _rbl:
                    _pp = state_cloth.pos.get(_rb)
                    if _pp is None:
                        continue
                    _best = None
                    for _col in cols_f:
                        _crbi = _col[-1]
                        # 実際の衝突判定と同じ絞り込みを行う。これを省くと、
                        # チェーンのアンカー剛体(構造上その内側にある)まで最寄り
                        # として拾ってしまい、貫通率が常時100%に見える
                        _pp_obj = _rep_part.get(_rb)
                        _ex = exclude_rb.get(_rb) if exclude_rb else None
                        if _ex is not None and _crbi in _ex:
                            continue
                        if allowed_collider_rbi is not None:
                            if _crbi not in allowed_collider_rbi:
                                continue
                        elif _pp_obj is not None and not (
                                (_col[-2] & (1 << _pp_obj.group))
                                and (_pp_obj.collision_mask & (1 << _col[-3]))):
                            continue
                        if _col[0] == "sphere":
                            _C, _rad = _col[1], _col[2]
                        else:
                            _C = _closest_on_segment(_pp, _col[1], _col[2])
                            _rad = _col[3]
                        _dd = _len(_sub(_pp, _C)) - _rad
                        if _best is None or _dd < _best[0]:
                            _best = (_dd, _crbi)
                    if _best is not None:
                        _e = _rep_near.setdefault((_k, _best[1]), [0, 0.0])
                        _e[0] += 1
                        _e[1] += _best[0]
                        # メッシュ貫通の推定: クリアランス < ドレープ深さ なら、
                        # ボーンは当たっていなくても布の谷はコライダーの内側にある
                        _dv = _rep_drape.get(_rb)
                        if _dv is not None:
                            _pe = _rep_pen.setdefault(_k, [0, 0, 0.0, 0.0])
                            _pe[0] += 1
                            if _best[0] < _dv:
                                _pe[1] += 1
                                _depth = _dv - _best[0]
                                _pe[2] += _depth
                                if _depth > _pe[3]:
                                    _pe[3] = _depth

        if _rep_rest:
            _bcf = body_axis_at(f)[0]
            for _k, _rbl in _rep_rings.items():
                _sum = 0.0
                _n = 0
                for _rb in _rbl:
                    _pp = state_cloth.pos.get(_rb)
                    if _pp is None:
                        continue
                    _sum += math.hypot(_pp[0] - _bcf[0], _pp[2] - _bcf[2]) - _rep_rest[_rb]
                    _n += 1
                if _n:
                    _rep_series[_k].append(_sum / _n)
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
            st = state_cloth if id(ch) in _cloth_chain_ids else state_other
            for k in range(1, len(plist)):
                c = plist[k]
                lq = loc.get(c.bone, (0.0, 0.0, 0.0, 1.0))
                if c.rb in skirt_rbs:
                    lt = _apply(_inv_rigid(pw), st.pos[c.rb])
                    if c.bone not in t_written:
                        tkeys.setdefault(c.bone, []).append((f, lt))
                        t_written.add(c.bone)
                    pw = mat_mul(pw, trs_to_mat(lt, lq))
                else:
                    rdir = st.rest_dir.get(c.rb, (0.0, 0.0, 0.0))
                    rl = c.rest_len
                    pw = mat_mul(pw, trs_to_mat((rdir[0]*rl, rdir[1]*rl, rdir[2]*rl), lq))

        prev_anchor_full = a
        prev_colliders_full = cols_f

    if _substep_enabled:
        print("  [physics] adaptive substep total: fired on %d/%d frames"
              % (_substep_fire_total, num_frames))

    # --- 計測結果: スカートのリングが静止時の半径からどれだけ開いたか ---
    # 「接触した瞬間だけ開くのか」「開いたまま戻らないのか」を数字で見るため、
    # 平均・最大に加えて、動画を10等分した区間ごとの平均も出す。
    def _rep_near_rings():
        return sorted({k[0] for k in _rep_near})

    if _rep_series and any(_rep_series.values()):
        print("  [physics] skirt ring expansion vs rest radius (glTF units, + = opened outward):")
        for _k in sorted(_rep_series):
            _v = _rep_series[_k]
            if not _v:
                continue
            _sv = sorted(_v)
            _p50 = _sv[len(_sv) // 2]
            _p95 = _sv[min(len(_sv) - 1, int(len(_sv) * 0.95))]
            _over = sum(1 for x in _v if x > 0.005) * 100.0 / len(_v)
            print("    ring %d (%2d bones): median %+.4f  p95 %+.4f  max %+.4f  min %+.4f  "
                  "frames over +0.005: %.0f%%"
                  % (_k, len(_rep_rings[_k]), _p50, _p95, max(_v), min(_v), _over))
        for _k in sorted(_rep_near_rings()):
            _all = [(v[0], k[1], v[1] / max(v[0], 1)) for k, v in _rep_near.items() if k[0] == _k]
            _tot = sum(x[0] for x in _all) or 1
            _rows = sorted(_all, reverse=True)[:3]
            if _rows:
                print("    ring %d nearest collider (sampled): %s"
                      % (_k, ", ".join("%s %d%% (mean surface dist %+.4f)"
                                       % (rbs_names_all[c]["name"] if 0 <= c < len(rbs_names_all)
                                          else c, round(100.0 * n / _tot), d)
                                       for n, c, d in _rows)))
        if _rep_pen:
            print("    (estimated mesh penetration = clearance below the measured "
                  "drape depth; the bone itself may still be clear)")
            for _k in sorted(_rep_pen):
                _n, _hit, _sum, _mx = _rep_pen[_k]
                print("    ring %d mesh penetration: %.1f%% of samples, mean depth %.4f, "
                      "max %.4f" % (_k, 100.0 * _hit / max(1, _n),
                                    _sum / max(1, _hit), _mx))
        for _k in sorted(_rep_rings):
            _rbl = _rep_rings[_k]
            _hit = [(_push_report.get(_rb) or [0, 0.0, 0.0]) for _rb in _rbl]
            _fr = sum(h[0] for h in _hit)
            if _fr:
                print("    ring %d pushes: %.1f%% of frames per bone, mean push %.4f, max %.4f"
                      % (_k, 100.0 * _fr / max(1, num_frames * len(_rbl)),
                         sum(h[1] for h in _hit) / _fr, max(h[2] for h in _hit)))
            else:
                print("    ring %d pushes: none" % _k)
        for _k in sorted(_rep_series):
            _v = _rep_series[_k]
            if len(_v) < 10:
                continue
            _seg = len(_v) // 10
            _tl = [sum(_v[i * _seg:(i + 1) * _seg]) / _seg for i in range(10)]
            print("    ring %d timeline: %s" % (_k, " ".join("%+.3f" % x for x in _tl)))

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
def _joint_lateral_slack(j):
    """joint の linearLimitMin/Max(既にglTFスケール済み)から、横距離拘束用の
    「遊び」量をスカラーで概算する。本家の6DOFジョイントは軸ごとの箱形制限
    だが、この簡易ソルバーは1本のスカラー距離拘束しか持たないため、各軸の
    絶対値の平均を代表値として使う(このモデルでは等方的な値が入っている
    ことを実測確認済み)。linearLimitMin/Maxが無い/取得できない場合は0。
    """
    mn = j.get("linearLimitMin")
    mx = j.get("linearLimitMax")
    if not mn or not mx:
        return 0.0
    vals = [abs(v) for v in list(mn) + list(mx)]
    return sum(vals) / len(vals) if vals else 0.0


def _joint_lateral_angle_info(j):
    """joint の angularLimitMin/Max/rotation(YXZオイラー角、既にglTFスケール
    済み)をそのまま(ang_min, ang_max, joint_rot)として返す。cloth_algorithm=
    "rigid_body"の横リング角度制限(_rb_solve_angle_limit)で、縦の関節と
    全く同じ仕組みで使う。距離拘束(_joint_lateral_slack)しか実装していな
    かった旧版は、横方向の「向き」を拘束しておらず各パネルの局所X軸が
    バラバラにずれてリングが崩れる不具合の原因になっていた(実機フィード
    バックで発覚)。角度制限データが取得できない場合はNoneを返す。
    """
    amin = j.get("angularLimitMin")
    amax = j.get("angularLimitMax")
    jrot = j.get("rotation")
    if not amin or not amax:
        return None, None, None
    return tuple(amin), tuple(amax), (tuple(jrot) if jrot else (0.0, 0.0, 0.0, 1.0))


def extract_chains_bfs(physics_gltf, bone_world_matrices, only_names=None,
                       bone_parents=None):
    """アンカー(mode0剛体)からのBFSで親を決め、縦チェーン＋横距離拘束に分解。
    髪(単純チェーン)もスカート(リング)も統一的に扱える。

    bone_parents: 省略可。node index -> 親node index(無ければ-1)のリスト。
        渡すと、ツリー辺の選択が辞書式Dijkstra(第1キー=ボーン階層と不整合な
        辺の本数、第2キー=3D距離)になる。「整合」とは、親剛体のボーンが
        子剛体ボーンの直接の親(または同一ボーン)であること。これにより、
        しっぽ先端を腰に繋ぎ直す「吊り」ジョイント等の近道エッジが、正規の
        鎖(全て整合辺=不整合0本)より優先されることがなくなる。吊り辺自体は
        非ツリー辺(横距離拘束)として残る。Noneなら従来通り3D距離のみ。

    戻り値: (chains, parts, excluded, lateral)
      lateral = [(rb_i, rb_j, rest_len, joint_slack, ang_min, ang_max, joint_rot), ...]
        非ツリー辺（横リング）の距離拘束。joint_slack は元のPMXジョイントの
        linearLimitMin/Maxから概算した「遊び」量(glTFスケール済み、常に0.0
        以上)。lateral_slack_scale=0.0の既定では使用側で無視されるため、
        この値自体が挙動に影響することはない。ang_min/ang_max/joint_rotは
        同じジョイントのangularLimitMin/Max/rotation(縦の関節のPartcle.
        ang_min/max/joint_rotと同じ形式)。cloth_algorithm="rigid_body"の
        横リング角度制限でのみ使用し、点ベース方式(_apply_lateral_constraint)
        はこれらを無視する(距離拘束のみで従来通り)。取得できない場合は
        (None, None, None)。
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
                     collision_mask=rbs[i].get("noCollisionMask", 0xFFFF))
        p.rest_pos = bpos(rbs[i]["bone"])
        parts[i] = p
    for i in anchors:
        p = Particle(i, rbs[i]["bone"], rbs[i]["mass"], kinematic=True,
                     group=rbs[i].get("group", 0),
                     collision_mask=rbs[i].get("noCollisionMask", 0xFFFF))
        p.rest_pos = bpos(rbs[i]["bone"])
        parts[i] = p

    # Dijkstra（全アンカー同時開始、辺重み=rest距離）→ 親(ツリー辺)確定、非ツリー辺=横拘束。
    # 単純なホップ数BFSだと、リング接続(同じ段の輪)を辿った方がアンカーから
    # 少ないホップで届くことがあり、その場合はリングを横断する不自然な親子関係
    # (例: 上段の輪を1周してから下段に接続)になりやすい。これは実際の物理挙動として
    # 不安定（アンカーの動きが輪全体に梃子のように伝わり、回転出力が特異点近くで
    # 跳ねる)なので、ホップ数ではなく実際の3D距離を最短路の重みにする。
    #
    # bone_parents が渡された場合はさらに、優先度を(不整合辺数, 3D距離)の
    # 辞書式にする。不整合辺 = 「親剛体のボーンが子剛体ボーンの直接の親
    # (または同一ボーン)」でない辺。しっぽ末端を腰に繋ぐ「吊り」ジョイントの
    # ような近道は3D距離では鎖経由より短くなり得るが(実例: Tda式ミク・アペンド
    # 改のしっぽ吊・腰ベルト６)、不整合辺を1本含むため、全て整合な正規の鎖に
    # 必ず負ける。
    def edge_clean(pa_rb, ch_rb):
        if bone_parents is None:
            return True
        ba = rbs[pa_rb]["bone"]
        bb = rbs[ch_rb]["bone"]
        if ba == bb:
            return True          # 同一ボーンを共有する剛体
        if 0 <= bb < len(bone_parents):
            return bone_parents[bb] == ba
        return False

    import heapq
    ZERO = (0, 0.0)
    depth = {i: ZERO for i in anchors}

    def better(nd, cur_d):
        # 辞書式比較。距離側は従来の 1e-9 イプシロン付き。
        if nd[0] != cur_d[0]:
            return nd[0] < cur_d[0]
        return nd[1] < cur_d[1] - 1e-9

    tree_joint = {}     # child_rb -> joint（親との辺）
    visited = set(anchors)
    lateral = []
    seen_pairs = set()
    heap = [(ZERO, i) for i in anchors]
    heapq.heapify(heap)
    while heap:
        dcur, cur = heapq.heappop(heap)
        if better(depth.get(cur, dcur), dcur):
            continue   # 古いエントリ（既により良い経路で確定済み）
        for nb, j in adj.get(cur, []):
            if nb not in parts:      # 対象外(別カテゴリのdyn等)は無視
                continue
            rp_c, rp_n = parts[cur].rest_pos, parts[nb].rest_pos
            w = _len(_sub(rp_c, rp_n)) if (rp_c and rp_n) else 1.0
            nd = (dcur[0] + (0 if edge_clean(cur, nb) else 1), dcur[1] + w)
            if nb not in visited:
                visited.add(nb)
                depth[nb] = nd
                parts[nb].parent = cur
                tree_joint[nb] = j
                heapq.heappush(heap, (nd, nb))
            elif better(nd, depth.get(nb, (1 << 30, float("inf")))):
                # より短い経路が後から見つかった（優先度キューの構造上あり得る）。
                # 親を更新し、旧親子辺は横拘束として残す。
                old_parent = parts[nb].parent
                if old_parent != -1 and old_parent != cur:
                    key = frozenset((old_parent, nb))
                    if key not in seen_pairs:
                        seen_pairs.add(key)
                        rp_a, rp_b = parts[old_parent].rest_pos, parts[nb].rest_pos
                        if rp_a and rp_b:
                            _old_j = tree_joint.get(nb)
                            _amin, _amax, _jrot = _joint_lateral_angle_info(_old_j) if _old_j else (None, None, None)
                            lateral.append((old_parent, nb, _len(_sub(rp_a, rp_b)),
                                            _joint_lateral_slack(_old_j) if _old_j else 0.0,
                                            _amin, _amax, _jrot))
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
                                _amin, _amax, _jrot = _joint_lateral_angle_info(j)
                                lateral.append((cur, nb, _len(_sub(rp_a, rp_b)),
                                                _joint_lateral_slack(j),
                                                _amin, _amax, _jrot))

    # rest_len / 角度制限 を tree_joint から埋める
    for c_rb, p in parts.items():
        if p.kinematic or p.parent == -1:
            continue
        j = tree_joint.get(c_rb)
        if j:
            p.ang_min = j["angularLimitMin"]
            p.ang_max = j["angularLimitMax"]
            p.joint_rot = j.get("rotation")
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
        members.sort(key=lambda i: depth.get(i, (0, 0.0)))
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
    for a, b, _rl, _sl, _amin, _amax, _jrot in lateral:
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
def _apply_lateral_constraint(pos, part, lateral, lateral_slack_scale):
    """横リング(隣接パネル間)の距離拘束。simulate_step_cloth・
    simulate_step_rigid_chain の両方から共有される(単一の実装)。

    lateral_slack_scale>0のとき、元のPMXジョイントが持つ「遊び」
    (joint_slack、linearLimitMin/Maxから概算)×この倍率の範囲内では
    無補正にする。本家は該当ジョイントのバネ定数が0(=遊びの範囲内は
    完全フリー、限界で硬く止まる)であり、この簡易ソルバーの従来実装
    (常にrlぴったりへ戻すハード等式拘束)が持つ「伸縮ゼロの硬い網」の
    感触を緩和するために追加。デフォルト0.0では常にtarget=rlとなり、
    既存の等式拘束と完全に同じ(回帰安全)。posを直接書き換える。
    """
    for a, b, rl, joint_slack, _amin, _amax, _jrot in lateral:
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
        slack = joint_slack * lateral_slack_scale
        if slack > 0.0:
            lo, hi = rl - slack, rl + slack
            if lo <= dist <= hi:
                continue   # 遊びの範囲内: 本家同様に無補正
            target = lo if dist < lo else hi
        else:
            target = rl
        corr = _scale(d, (dist - target) / dist)
        pos[a] = _add(pos[a], _scale(corr, wa / ws))
        pos[b] = _sub(pos[b], _scale(corr, wb / ws))


def simulate_step_cloth(state, gravity_dir, dt, drag_force, stiffness_force,
                        lateral, gravity_power=0.02, iterations=6,
                        colliders=None, body_axis=None, radial_rbs=None,
                        margin=0.0, exclude_rb=None, skip_angle_clamp_rbs=None,
                        allowed_collider_rbi=None, hem_weight=None, hem_extra_margin=0.0,
                        collision_bounce=0.0, extra_margin_by_rb=None,
                        margin_cap_by_pair=None, collision_velocity_damp=1.0,
                        push_report=None, push_preserve_length=True,
                        skip_embedded_parent=True,
                        lateral_slack_scale=0.0, vertical_spread_scale=0.0,
                        vertical_neighbors=None, segment_aware_collision=False,
                        hem_extend=None):
    """縦チェーン＋横距離拘束を反復して解くクロスソルバ。
    lateral: [(rb_i, rb_j, rest_len, joint_slack), ...]
    segment_aware_collision: Trueだと衝突判定をパーティクル単体でなく
      「親→子の区間」に対して行い(resolve_collisions_segments)、衝突点が
      区間の途中にあるときは押し出しを親・子へ按分する。既定Falseで
      既存挙動(resolve_collisions、パーティクル単体判定)と完全に同じ。
    hem_extend: {終端パーティクルrb: 延長長} の辞書。segment_aware_collision=True
      のときのみ使用(裾延長。詳細はbake_hair_into_gltfのhem_extend_scale参照)。
      None/空(デフォルト)なら既存挙動と完全に同じ。
    戻り値: seg_rot[rb]
    """
    pos, prev, part, rest_dir = state.pos, state.prev, state.part, state.rest_dir
    anchor_rot = getattr(state, "_anchor_rot", {})
    last_seg = getattr(state, "_last_seg", {})
    stiff = stiffness_force * dt
    # 安全弁: このフレーム開始時点の位置を記録（最大変位クランプ用）
    frame_start_pos = dict(pos)
    # 弾み(collision_bounce)用: このフレーム中にresolve_collisionsが実際に
    # 押し出した変位を蓄積する。反復の各回・念押しの1回、いずれの押し出しも
    # ここに積算し、フレームの最後に一度だけその合計へ弾み係数を掛けて適用する
    # (詳細はresolve_collisionsのaccum引数のdocstring参照)。collision_bounce=0
    # ならNoneのままにして、その場合は既存挙動と完全に同じにする。
    # collision_bounce(弾み)とvertical_spread_scale(縦方向への押し出し分配)は
    # どちらも「そのフレーム中にresolve_collisionsが実際に押し出した変位の
    # 合計」を必要とするため、同じaccum機構を共用する。
    _bounce_accum = ({} if (collision_bounce or vertical_spread_scale
                            or collision_velocity_damp) else None)
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
        # 横: 距離拘束（両者dynamicなのでinv_mass比で配分）。詳細は
        # _apply_lateral_constraint() のdocstring参照。
        _apply_lateral_constraint(pos, part, lateral, lateral_slack_scale)
        # コライダー衝突（脚カプセル等への push-out）
        if colliders:
            # 弾みはここでは直接適用しない(accumへ積算するだけ)。この呼び出しは
            # 1フレームにつき最大iterations回(既定6回)実行される反復拘束解決
            # の一部で、都度オーバーシュートを適用すると呼び出し回数分複利的に
            # 積み重なり値を大きくすると容易に破綻する。実際の適用はフレームの
            # 最後に一度だけ、accumに集計された合計変位に対して行う。
            if segment_aware_collision:
                resolve_collisions_segments(state, state.chains, colliders, body_axis=body_axis,
                                            radial_rbs=radial_rbs, margin=margin,
                                            exclude_rb=exclude_rb,
                                            allowed_collider_rbi=allowed_collider_rbi,
                                            hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                                            accum=_bounce_accum, extra_margin_by_rb=extra_margin_by_rb,
                                            margin_cap_by_pair=margin_cap_by_pair,
                                            hem_extend=hem_extend,
                                            push_preserve_length=push_preserve_length,
                                            skip_embedded_parent=skip_embedded_parent)
            else:
                resolve_collisions(state, colliders, body_axis=body_axis,
                                   radial_rbs=radial_rbs, margin=margin,
                                   exclude_rb=exclude_rb,
                                   allowed_collider_rbi=allowed_collider_rbi,
                                   hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                                   accum=_bounce_accum, extra_margin_by_rb=extra_margin_by_rb,
                                   margin_cap_by_pair=margin_cap_by_pair)
        # アンカーは常に固定位置へ（set_anchor で pos は既に固定済み）

    # 反復後にもう一度押し出し: 直前の length 拘束が侵入を復活させても、
    # 記録される最終位置は必ずコライダー外になるようにする（修正2）。
    if colliders:
        if segment_aware_collision:
            resolve_collisions_segments(state, state.chains, colliders, body_axis=body_axis,
                                        radial_rbs=radial_rbs, margin=margin,
                                        exclude_rb=exclude_rb,
                                        allowed_collider_rbi=allowed_collider_rbi,
                                        hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                                        accum=_bounce_accum, extra_margin_by_rb=extra_margin_by_rb,
                                        margin_cap_by_pair=margin_cap_by_pair,
                                        hem_extend=hem_extend,
                                        push_preserve_length=push_preserve_length,
                                        skip_embedded_parent=skip_embedded_parent)
        else:
            resolve_collisions(state, colliders, body_axis=body_axis,
                               radial_rbs=radial_rbs, margin=margin,
                               exclude_rb=exclude_rb,
                               allowed_collider_rbi=allowed_collider_rbi,
                               hem_weight=hem_weight, hem_extra_margin=hem_extra_margin,
                               accum=_bounce_accum, extra_margin_by_rb=extra_margin_by_rb,
                                   margin_cap_by_pair=margin_cap_by_pair)

        # vertical_spread_scale: コライダーに当たって押し出された1パーティクルの
        # 変位を、縦方向に隣接する1つ上・1つ下のパーティクル(1ホップのみ、
        # kinematicアンカーは対象外)へも分配する。実機フィードバック(片足を
        # 上げるポーズでスカートに「輪切り」の段が出る)への対策。フレームに
        # つき一度だけ、そのフレーム全体の押し出し合計(_bounce_accum)を使う。
        # 修正3(長さ拘束の再クランプ)より前に置くことで、分配した結果は
        # 伸縮ではなく「向き(傾き)」の変化として隣接パーティクルへ伝わる。
        # デフォルト0.0では_bounce_accumがNoneのまま(collision_bounceも無効な
        # 限り)か、このブロック自体がスキップされ既存挙動と完全に同じ。
        #
        # 重要: posだけを動かしprevを動かさないと、Verlet積分の速度項
        # (pos-prev)にこの分配ぶんがそのまま「速度」として乗ってしまう。
        # 通常の衝突押し出しは1回限りのイベントなら問題にならないが、
        # 脚の近くで裾が常時軽く触れているような場面(実測: このモデル・
        # モーションでは全7001フレーム中7272回、ほぼ毎フレーム発火)では、
        # 速度注入が減衰し切る前に毎フレーム上書きされ続け、何もない場面
        # でもスカートが暴れる原因になっていた(実機フィードバックで確認)。
        # collision_bounce(意図的に速度を注入する機能)とは異なり、この
        # 分配はあくまで「向きを滑らかにする」だけが目的で速度を足す
        # 意図が無いため、prevも同じ量だけ動かして(pos-prev)を変えない
        # ようにし、速度注入を起こさない「その場の位置修正」に変更する。
        if vertical_spread_scale > 0.0 and _bounce_accum and vertical_neighbors:
            for rb, disp in list(_bounce_accum.items()):
                for nb_rb in vertical_neighbors.get(rb, ()):
                    nb_part = part.get(nb_rb)
                    if nb_part is None or nb_part.inv_mass <= 0:
                        continue
                    d = _scale(disp, vertical_spread_scale)
                    pos[nb_rb] = _add(pos[nb_rb], d)
                    prev[nb_rb] = _add(prev[nb_rb], d)

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

    # 弾み(collision_bounce)の適用: このフレーム中にresolve_collisionsが
    # 実際に押し出した変位の"合計"(_bounce_accum、反復6回+念押し1回の全て)を
    # 使って、フレームにつき一度だけオーバーシュートを与える。修正3(rest_len
    # 再クランプ)より後に置くことが重要: 修正3より前に適用すると、スカート等
    # (radial_rbs)ではrest_len再クランプで方向以外の変化がほぼ消されてしまい、
    # 弾みが見た目にほとんど効かなくなる。また念押し1回分の変位だけでは反復
    # 拘束がほぼ収束済みで小さすぎるため、6回の反復+念押し全ての合計を見て
    # 初めて弾みとして意味のある大きさになる。
    # 計測: このフレームで実際に押し出しが起きたパーティクルと、その大きさ
    if push_report is not None and _bounce_accum:
        for rb, disp in _bounce_accum.items():
            _e = push_report.setdefault(rb, [0, 0.0, 0.0])
            _e[0] += 1
            _dl = _len(disp)
            _e[1] += _dl
            if _dl > _e[2]:
                _e[2] = _dl

    if _bounce_accum:
        for rb, disp in _bounce_accum.items():
            if part[rb].inv_mass <= 0:
                continue
            if collision_bounce:
                pos[rb] = _add(pos[rb], _scale(disp, collision_bounce))
                prev[rb] = _sub(prev[rb], _scale(disp, collision_bounce))
            # 押し出しは pos だけを動かすため、Verlet では「押された分だけ
            # 外向きの速度を得た」ことになる(次フレームの慣性 pos-prev に乗る)。
            # 毎フレーム接触していると、この速度が供給され続けて布が外向きへ
            # 送られ、離れても戻らない(実測: IAのスカートは全フレームで
            # 腰リング+0.025のまま平坦、コライダーを外すと中央-0.001)。
            # prev を同じだけ動かして、押し出しが速度を生まないようにする。
            # 1.0=速度を完全に打ち消す(既定)、0.0=従来どおり速度が乗る。
            if collision_velocity_damp:
                prev[rb] = _add(prev[rb], _scale(disp, collision_velocity_damp))



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
# 【実験的・独立実装】板(剛体チェーン)方式 — cloth_algorithm="rigid_chain"
# ======================================================================
# 既存の simulate_step_cloth は各段を「位置だけを持つ点」として扱い、回転は
# 「結果として位置がどちらを向いたか」から逆算するだけ（独立した物理量ではない）。
# 本家MMD(Bullet)は逆に、縦ジョイントの移動を完全固定する代わりに回転を
# ジョイント制限の範囲で自由に持たせ、パネルが脚の角度に合わせて傾くことで
# 滑らかな曲線を作っている(実測: X軸±80°/Y軸±5°/Z軸±10°、バネ定数0＝
# 範囲内フリー・限界で硬く止まる)。
## この関数は、既存コードの慣習(rest_dir/tgt_dir/aimの連鎖規則、q_mul(aim,q_par)
# でのトップダウン合成)をそのまま踏襲しつつ、各パーティクルの「世界姿勢
# そのもの」(world_q)を、位置から毎フレーム作り直すのではなく、慣性・
# 重力トルク・stiffness緩和を持つ独立した状態として持続させる。慣性は
# 世界姿勢の変化量で計算する(親相対の偏差ではない)。これにより、親(腰)
# が急に速く回転したときに子(裾)が慣性でその場に留まろうとして遅れ、
# 遠心力で外側へ流れる効果が自然に出る(親相対の偏差だけで慣性を持つと、
# 親の回転にほぼ無抵抗で追従してしまい遠心力が出ない設計ミスを実機
# フィードバックで発見・修正した経緯がある)。位置は最後にこの姿勢から
# 導出する(simulate_step_clothとは主従が逆)。
#
# Stage 1のスコープ(ユーザーと合意): 縦方向の回転駆動のみ。衝突・横リング・
# 適応サブステップ・中間パーティクル等は含めない、既存パイプラインには
# まだ組み込まない独立関数。まずこれ単体で「重力で自然に垂れ下がり、
# アンカーの動きに滑らかに追従する」挙動になるかを検証する。
# 角度制限は本家ジョイントが実際に持つX/Y/Z軸別の値(joint_rotで定義された
# フレーム、YXZオイラー角)をそのまま使う。既存コードの等方コーン近似
# (|X|と|Z|の大きい方だけを見る、Y軸±5°等の厳しい軸を無視する簡易版)から
# 一歩進め、本家の「ピッチは自由・ヨーロールは厳しい」という非対称な
# 制限をそのまま再現する。
def _quat_axis_angle(axis, angle):
    """軸(単位ベクトル)・角度(ラジアン)からクォータニオンを作る。"""
    s = math.sin(angle * 0.5)
    return (axis[0]*s, axis[1]*s, axis[2]*s, math.cos(angle * 0.5))


def _quat_to_axis_angle(q):
    """クォータニオンを(軸, 角度[ラジアン])に分解する。角度は0〜2πの実回転角。
    ほぼ無回転(角度≈0)の場合は軸を(0,0,1)とし、以後の計算で無害にする。
    """
    x, y, z, w = q
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w*w))
    if s < 1e-9:
        return (0.0, 0.0, 1.0), 0.0
    return (x/s, y/s, z/s), angle


def _quat_to_euler_yxz(q):
    """クォータニオンを、physics.pyのeuler_to_quat(order='YXZ')と対になる
    (rx, ry, rz)[ラジアン]へ分解する(R = Ry(ry)*Rx(rx)*Rz(rz)の意味で対応)。
    MMDジョイントのangularLimitMin/MaxはこのYXZ順のオイラー角として定義
    されているため、軸別クランプにはこの順序での分解が必要。
    定数はthree.js Euler.setFromRotationMatrix(order='YXZ')と同型の
    標準的な導出式(ジンバルロック近傍のみ特別扱い)。
    """
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    m11 = 1 - 2*(yy+zz); m13 = 2*(xz+wy)
    m21 = 2*(xy+wz);     m22 = 1 - 2*(xx+zz); m23 = 2*(yz-wx)
    m31 = 2*(xz-wy);     m33 = 1 - 2*(xx+yy)
    m23c = max(-1.0, min(1.0, m23))
    rx = math.asin(-m23c)
    if abs(m23c) < 0.9999999:
        ry = math.atan2(m13, m33)
        rz = math.atan2(m21, m22)
    else:
        ry = math.atan2(-m31, m11)
        rz = 0.0
    return rx, ry, rz


def simulate_step_rigid_chain(chains, rest_dir, rest_len, part, pos, anchor_rot,
                              world_q, prev_world_q, gravity_dir, dt, drag_force,
                              stiffness_force, gravity_power=0.02):
    """板(剛体チェーン)方式の1ステップ。縦方向の回転駆動のみ(Stage 1/2)。

    引数(既存のPBD系関数とは呼び出し形が異なる。独立実装のため):
      chains    : extract_chains_bfs() が返す ChainInfo のリスト
      rest_dir  : {rb: 単位ベクトル} 親からのrest方向(world/bind基準、既存と同じ規約)
      rest_len  : {rb: float} 親からの静止距離(通常 part[rb].rest_len と同じ)
      part      : {rb: Particle}
      pos       : {rb: (x,y,z)} アンカー(kinematic)のワールド位置は呼び出し側が
                  事前にセット済みであること(既存のset_anchor相当)。この関数は
                  動的パーティクルのposをここに書き込んで返す(戻り値ではなく
                  このdictを直接更新する)。
      anchor_rot: {rb: quat} アンカーボーンのワールド回転(既存の_anchor_rotと同じ)
      world_q, prev_world_q: {rb: quat} 各パーティクルの「世界姿勢そのもの」と
                  前フレーム値。呼び出し側が辞書を保持し続けること(初回は
                  空dictを渡せば親の回転で初期化される)。
                  重要: 慣性はこの世界姿勢の変化量で計算する(親相対の偏差
                  ではない)。親相対の偏差で慣性を計算すると、親(腰)が
                  急に速く回転したときに子(裾)がほぼ無抵抗で追従して
                  しまい、遠心力で外側へ流れる効果が原理的に出ない
                  (実機フィードバックで発覚したバグの修正)。世界姿勢基準
                  なら、子は「前フレームまでの世界姿勢を慣性で維持しよう
                  とする」ため、親の急な回転に対して自然に遅れ、結果として
                  外側へ流れる(遠心力)効果が出る。
      戻り値    : seg_rot {rb: quat}（既存のsimulate_step_cloth等と同じ形式、
                  そのままbake_hair_into_gltfの出力パイプラインに渡せる）
    """
    seg_rot = {}
    stiff = max(0.0, min(1.0, stiffness_force * dt))   # 1フレームで戻す割合(0..1)
    grav_ang = gravity_power * dt                       # 重力による回転角(rad)
    damp = max(0.0, min(1.0, 1.0 - drag_force))          # 角速度の残存率

    for ch in chains:
        plist = ch.particles
        q_par = anchor_rot.get(plist[0].rb, (0.0, 0.0, 0.0, 1.0)) if plist[0].kinematic \
            else (0.0, 0.0, 0.0, 1.0)
        for k in range(1, len(plist)):
            c = plist[k]; pa = plist[k-1]
            rdir = rest_dir.get(c.rb)
            if rdir is None or c.inv_mass <= 0:
                # kinematicや情報欠落時は既存位置から向きを引くだけ(髪等と同じ扱い)
                cur_dir = _norm(_sub(pos[c.rb], pos[pa.rb])) if c.rb in pos else (0.0, -1.0, 0.0)
                aim = q_from_to(q_rotate_vec(q_par, rdir) if rdir else cur_dir, cur_dir)
                seg_rot[c.rb] = q_mul(aim, q_par)
                q_par = seg_rot[c.rb]
                continue

            wq = world_q.get(c.rb)
            if wq is None:
                wq = q_par   # 初回シード: 偏差ゼロ(tgt_dirと一致)から開始
            pwq = prev_world_q.get(c.rb, wq)

            # 1) 慣性: 前フレームの「世界姿勢そのもの」の変化量を damp 倍で
            #    持ち越す(親相対ではなく世界基準。上のdocstring参照)。
            delta = q_mul(wq, q_conj(pwq))
            axis, ang = _quat_to_axis_angle(delta)
            if ang > math.pi:      # 最短経路(実回転角0〜π)に正規化
                ang = 2.0 * math.pi - ang
                axis = tuple(-a for a in axis)
            inertia_q = _quat_axis_angle(axis, ang * damp) if ang > 1e-9 else (0.0, 0.0, 0.0, 1.0)
            prev_world_q[c.rb] = wq
            wq = q_mul(inertia_q, wq)

            # 2) 重力トルク: 世界姿勢を直接、現在の実方向からgravity_dirへ
            #    わずかに回す。
            cur_dir = q_rotate_vec(wq, rdir)
            if grav_ang:
                axis_g = _cross(cur_dir, gravity_dir)
                if _len(axis_g) > 1e-9:
                    axis_g = _norm(axis_g)
                    wq = q_mul(_quat_axis_angle(axis_g, grav_ang), wq)
                    cur_dir = q_rotate_vec(wq, rdir)

            # 3) stiffness: 親相対の偏差(dev = wq を q_par 基準に見たもの)を
            #    rest(恒等)へ向けてstiff割合だけ緩和する。restは「親の現在
            #    の姿勢に対して」定義されるので、ここだけは親相対で扱う。
            if stiff > 0.0:
                dev = q_mul(wq, q_conj(q_par))
                axis_s, ang_s = _quat_to_axis_angle(dev)
                if ang_s > math.pi:
                    ang_s = 2.0 * math.pi - ang_s
                    axis_s = tuple(-a for a in axis_s)
                dev = _quat_axis_angle(axis_s, ang_s * (1.0 - stiff)) if ang_s > 1e-9 else (0.0, 0.0, 0.0, 1.0)
                wq = q_mul(dev, q_par)
                cur_dir = q_rotate_vec(wq, rdir)

            # 4) 角度制限: 本家ジョイントが実際に持つX/Y/Z軸別の制限を、その
            #    まま軸別クランプとして使う。親相対の偏差devを、ジョイント
            #    自身のフレーム(joint_rot、子ボーンのbindローカル基準)へ
            #    変換してからYXZオイラー角に分解しクランプ、逆変換で戻す。
            if c.ang_max and c.ang_min:
                jr = c.joint_rot or (0.0, 0.0, 0.0, 1.0)
                dev = q_mul(wq, q_conj(q_par))
                in_joint = q_mul(q_conj(jr), q_mul(dev, jr))
                rx, ry, rz = _quat_to_euler_yxz(in_joint)
                rx_c = max(c.ang_min[0], min(c.ang_max[0], rx))
                ry_c = max(c.ang_min[1], min(c.ang_max[1], ry))
                rz_c = max(c.ang_min[2], min(c.ang_max[2], rz))
                if (rx_c, ry_c, rz_c) != (rx, ry, rz):
                    in_joint_clamped = euler_to_quat(rx_c, ry_c, rz_c)
                    dev_clamped = q_mul(jr, q_mul(in_joint_clamped, q_conj(jr)))
                    wq = q_mul(dev_clamped, q_par)
                    cur_dir = q_rotate_vec(wq, rdir)

            world_q[c.rb] = wq
            pos[c.rb] = _add(pos[pa.rb], _scale(cur_dir, rest_len.get(c.rb, c.rest_len)))
            seg_rot[c.rb] = wq
            q_par = wq

    return seg_rot


# ======================================================================
# 【実験的・独立実装 Stage 3】剛体(位置+姿勢+並進速度+角速度)方式
# cloth_algorithm="rigid_body"
# ======================================================================
# simulate_step_rigid_chain(姿勢のみを状態に持つ)には並進の運動量が無く、
# 回転しても遠心力で広がらないという実機フィードバックを受けて調査した結果、
# Bullet(本家)のbtRigidBody::integrateVelocitiesはm_linearVelocity(並進)と
# m_angularVelocity(角速度)を両方独立に積分していることが判明。真の遠心力
# には並進の運動量が不可欠なため、XPBD(Position Based Dynamics with
# orientations、Müller et al. 2020)の考え方で「位置+姿勢+並進速度+角速度」
# を持つ剛体チェーンとして全面的に作り直したもの。
#
# 各スカートパネルを1つの剛体として扱う:
#   pos: 重心のワールド位置。rot: 重心のワールド姿勢(ローカル+Yが常に
#   「親方向」を向く規約)。half_len: 親秒側/子側の接続点までのローカル
#   Y距離。half_extents: 剛体(箱)のサイズ(PMX実測)、質量とあわせて
#   慣性テンソルを計算する。
#
# 拘束は3種類、いずれも反復(Gauss-Seidel)で解く:
#   1) 縦の点拘束(親の子側接続点=子の親側接続点)
#   2) 縦の角度制限(本家ジョイント実測のX/Y/Z軸別、joint_rot基準)
#   3) 横の距離拘束(隣接パネル間、lateral_slack_scaleと同じ「遊び」概念)
# 拘束を解いた後、位置/姿勢の変化から速度を再計算する(XPBDの要点)。
def _rb_box_inv_inertia(mass, half_extents):
    if mass <= 1e-9:
        return (0.0, 0.0, 0.0)
    w, h, d = half_extents
    ix = max((mass/3.0)*(h*h+d*d), 1e-9)
    iy = max((mass/3.0)*(w*w+d*d), 1e-9)
    iz = max((mass/3.0)*(w*w+h*h), 1e-9)
    return (1.0/ix, 1.0/iy, 1.0/iz)


class RigidBodyXPBD:
    __slots__ = ("pos", "rot", "vel", "omega", "kinematic", "inv_mass",
                 "inv_inertia_body", "half_len", "half_w",
                 "attach_parent_local", "attach_child_local",
                 "ang_min", "ang_max", "joint_rot", "rb",
                 "lin_damping", "ang_damping")

    def __init__(self, rb, pos, rot, mass, half_extents, half_len,
                kinematic=False, ang_min=None, ang_max=None, joint_rot=None,
                lin_damping=0.1, ang_damping=0.3):
        self.rb = rb
        self.pos = pos; self.rot = rot
        self.vel = (0.0, 0.0, 0.0); self.omega = (0.0, 0.0, 0.0)
        self.kinematic = kinematic
        self.inv_mass = 0.0 if (kinematic or mass <= 1e-9) else 1.0/mass
        self.inv_inertia_body = (0.0, 0.0, 0.0) if kinematic else _rb_box_inv_inertia(mass, half_extents)
        self.half_len = half_len
        self.half_w = half_extents[0] if half_extents else 0.01
        self.attach_parent_local = (0.0, half_len, 0.0)
        self.attach_child_local = (0.0, -half_len, 0.0)
        self.ang_min = ang_min
        self.ang_max = ang_max
        self.joint_rot = joint_rot or (0.0, 0.0, 0.0, 1.0)
        # PMXの各剛体が実際に持つ移動減衰・回転減衰(実測: このIAモデルの
        # スカートは0.9/0.9、以前の既定値0.1/0.3とは大きく異なっていた)。
        # rigid_body_linear/angular_dampingパラメータは、この実測値に掛ける
        # 倍率(既定1.0=モデルの値をそのまま使う)として使う。
        self.lin_damping = lin_damping
        self.ang_damping = ang_damping


def _rb_integrate_predict(body, gravity, dt, lin_damp_scale, ang_damp_scale):
    if body.kinematic:
        return body.pos, body.rot
    lin_damp = max(0.0, min(1.0, body.lin_damping * lin_damp_scale))
    ang_damp = max(0.0, min(1.0, body.ang_damping * ang_damp_scale))
    # 本家Bullet(btRigidBody::applyDamping)と同じ指数関数形式: 1秒あたりの
    # 除去割合(damping)を、実際のdtに対して (1-damping)^dt で減衰させる。
    # 以前は「1フレームでdamping分だけ除去」という線形近似(1-damping*dt*30)
    # を使っており、damping=0.9(このIAモデルの実測値)のような大きい値では
    # 正しい残存率92.6%/フレームに対し誤って10%/フレームまで減衰させて
    # しまっていた(実質ほぼ毎フレーム速度が消え、剛体が実質静止した状態に
    # 固まる不具合の原因。実機フィードバックで発覚)。
    vel = _add(body.vel, _scale(gravity, dt))
    vel = _scale(vel, (1.0 - lin_damp) ** dt)
    pos_pred = _add(body.pos, _scale(vel, dt))
    om = _scale(body.omega, (1.0 - ang_damp) ** dt)
    dq = q_mul((om[0]*dt*0.5, om[1]*dt*0.5, om[2]*dt*0.5, 0.0), body.rot)
    rot_pred = q_normalize(_add4(body.rot, dq))
    return pos_pred, rot_pred


def _add4(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3])


def _rb_solve_point_constraint(pA, rA, invmA, invIA, kinA, ptA_local,
                               pB, rB, invmB, invIB, kinB, ptB_local):
    wA = _add(pA, q_rotate_vec(rA, ptA_local))
    wB = _add(pB, q_rotate_vec(rB, ptB_local))
    C = _sub(wB, wA)
    Cl = _len(C)
    if Cl < 1e-12:
        return pA, rA, pB, rB
    n = _scale(C, 1.0/Cl)
    rA_ = q_rotate_vec(rA, ptA_local)
    rB_ = q_rotate_vec(rB, ptB_local)
    rAxn = _cross(rA_, n); rBxn = _cross(rB_, n)
    angA = sum(rAxn[i]**2*invIA[i] for i in range(3)) if not kinA else 0.0
    angB = sum(rBxn[i]**2*invIB[i] for i in range(3)) if not kinB else 0.0
    w_sum = invmA + invmB + angA + angB
    if w_sum <= 0:
        return pA, rA, pB, rB
    lam = -Cl / w_sum
    p = _scale(n, lam)
    if not kinA:
        pA = _sub(pA, _scale(p, invmA))
        rxp = _cross(rA_, p)
        dq = tuple(-0.5*invIA[i]*rxp[i] for i in range(3)) + (0.0,)
        rA = q_normalize(_add4(rA, q_mul(dq, rA)))
    if not kinB:
        pB = _add(pB, _scale(p, invmB))
        rxp = _cross(rB_, p)
        dq = tuple(0.5*invIB[i]*rxp[i] for i in range(3)) + (0.0,)
        rB = q_normalize(_add4(rB, q_mul(dq, rB)))
    return pA, rA, pB, rB


def _rb_slerp(a, b, t):
    d = _dot(a, b)
    if d < 0:
        b = tuple(-c for c in b); d = -d
    if d > 0.9995:
        return q_normalize(tuple(a[i] + (b[i]-a[i])*t for i in range(4)))
    theta0 = math.acos(max(-1.0, min(1.0, d)))
    theta = theta0 * t
    b2 = q_normalize(tuple(b[i] - a[i]*d for i in range(4)))
    return tuple(a[i]*math.cos(theta) + b2[i]*math.sin(theta) for i in range(4))


def _rb_solve_angle_limit(rA, rB, invIA, invIB, kinA, kinB, ang_min, ang_max, joint_rot):
    if ang_min is None or ang_max is None:
        return rA, rB
    rel = q_mul(rB, q_conj(rA))
    in_joint = q_mul(q_conj(joint_rot), q_mul(rel, joint_rot))
    rx, ry, rz = _quat_to_euler_yxz(in_joint)
    rx_c = max(ang_min[0], min(ang_max[0], rx))
    ry_c = max(ang_min[1], min(ang_max[1], ry))
    rz_c = max(ang_min[2], min(ang_max[2], rz))
    if (rx_c, ry_c, rz_c) == (rx, ry, rz):
        return rA, rB
    in_joint_c = euler_to_quat(rx_c, ry_c, rz_c)
    rel_c = q_mul(joint_rot, q_mul(in_joint_c, q_conj(joint_rot)))
    wA = 0.0 if kinA else sum(invIA)
    wB = 0.0 if kinB else sum(invIB)
    wsum = wA + wB
    if wsum <= 0:
        return rA, rB
    tB = wB / wsum; tA = wA / wsum
    rB_full = q_mul(rel_c, rA)
    rA_full = q_mul(q_conj(rel_c), rB)
    if not kinB and tB > 0:
        rB = q_normalize(_rb_slerp(rB, rB_full, tB))
    if not kinA and tA > 0:
        rA = q_normalize(_rb_slerp(rA, rA_full, tA))
    return rA, rB


def _rb_solve_distance_constraint(pA, rA, invmA, invIA, kinA, ptA_local,
                                  pB, rB, invmB, invIB, kinB, ptB_local,
                                  rest_dist, slack):
    wA = _add(pA, q_rotate_vec(rA, ptA_local))
    wB = _add(pB, q_rotate_vec(rB, ptB_local))
    C = _sub(wB, wA)
    dist = _len(C)
    if dist < 1e-12:
        return pA, rA, pB, rB
    lo, hi = rest_dist - slack, rest_dist + slack
    if lo <= dist <= hi:
        return pA, rA, pB, rB
    target = lo if dist < lo else hi
    n = _scale(C, 1.0/dist)
    Cerr = dist - target
    rA_ = q_rotate_vec(rA, ptA_local)
    rB_ = q_rotate_vec(rB, ptB_local)
    rAxn = _cross(rA_, n); rBxn = _cross(rB_, n)
    angA = sum(rAxn[i]**2*invIA[i] for i in range(3)) if not kinA else 0.0
    angB = sum(rBxn[i]**2*invIB[i] for i in range(3)) if not kinB else 0.0
    w_sum = invmA + invmB + angA + angB
    if w_sum <= 0:
        return pA, rA, pB, rB
    lam = -Cerr / w_sum
    p = _scale(n, lam)
    if not kinA:
        pA = _sub(pA, _scale(p, invmA))
        rxp = _cross(rA_, p)
        dq = tuple(-0.5*invIA[i]*rxp[i] for i in range(3)) + (0.0,)
        rA = q_normalize(_add4(rA, q_mul(dq, rA)))
    if not kinB:
        pB = _add(pB, _scale(p, invmB))
        rxp = _cross(rB_, p)
        dq = tuple(0.5*invIB[i]*rxp[i] for i in range(3)) + (0.0,)
        rB = q_normalize(_add4(rB, q_mul(dq, rB)))
    return pA, rA, pB, rB


def _rb_solve_min_separation(pA, invmA, kinA, pB, invmB, kinB, min_dist):
    """隣接する2つのスカート剛体の重心が、min_distより近づかないよう押し離す
    (球近似の剛体間衝突、回転は変えず並進のみ)。遠ざかる分には何もしない
    (最小距離のみの片側拘束)。バネの無いジョイントだけでは重力下で全パネル
    が「なるべく鉛直に垂れる」方向へ収束し扇形を保てない(実機フィードバック
    で発覚)問題への対策。本家も隣接パネル同士の衝突を持っていると推測される。
    """
    C = _sub(pB, pA)
    dist = _len(C)
    if dist >= min_dist or dist < 1e-9:
        return pA, pB
    n = _scale(C, 1.0 / dist)
    err = min_dist - dist
    w_sum = invmA + invmB
    if w_sum <= 0:
        return pA, pB
    lam = err / w_sum
    push = _scale(n, lam)
    if not kinA:
        pA = _sub(pA, _scale(push, invmA))
    if not kinB:
        pB = _add(pB, _scale(push, invmB))
    return pA, pB


def _rb_solve_collision_pos(pos, colliders, margin):
    """剛体の重心を脚コライダーの外へ押し出す(簡易版: 剛体の厚みは考慮
    しない点ベースの押し出しで、回転は変えず並進のみ。既存のresolve_
    collisionsと同じcolliders形式を使う。衝突が無ければposをそのまま返す。
    """
    best_push = None
    best_depth = 0.0
    for col in colliders:
        kind = col[0]
        if kind == "capsule":
            a, b, radius = col[1], col[2], col[3]
            c = _closest_on_segment(pos, a, b)
        elif kind == "sphere":
            c, radius = col[1], col[2]
        else:
            continue
        d = _sub(pos, c)
        dist = _len(d)
        depth = radius + margin - dist
        if depth > best_depth:
            best_depth = depth
            best_push = _scale(d, depth / dist) if dist > 1e-9 else (0.0, margin, 0.0)
    if best_push:
        return _add(pos, best_push)
    return pos


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

def _closest_on_segment_t(p, a, b):
    """_closest_on_segmentと同じだが、区間パラメータt(0=a側、1=b側)も返す。
    セグメント衝突の押し出しをa/bへ按分する際に使う。
    """
    ab = _sub(b, a)
    denom = _dot(ab, ab)
    if denom < 1e-12:
        return a, 0.0
    t = _dot(_sub(p, a), ab) / denom
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    return _add(a, _scale(ab, t)), t

def _closest_segment_segment_t(p1, q1, p2, q2):
    """2本の3D線分[p1,q1]・[p2,q2]間の最近接点対とパラメータを返す。
    戻り値: (t1, t2) — セグメント1・セグメント2それぞれの位置(0=始点,1=終点)。
    標準的なアルゴリズム(Ericson "Real-Time Collision Detection" 5.1.9)の実装。
    パーティクルの「区間」(親→子)がコライダーの軸(カプセル)のどのあたりに
    最接近しているかを求め、押し出しを親・子へ按分するために使う。
    """
    d1 = _sub(q1, p1)   # セグメント1の方向
    d2 = _sub(q2, p2)   # セグメント2の方向
    r = _sub(p1, p2)
    a = _dot(d1, d1)
    e = _dot(d2, d2)
    f = _dot(d2, r)
    if a < 1e-12 and e < 1e-12:
        return 0.0, 0.0
    if a < 1e-12:
        t1 = 0.0
        t2 = max(0.0, min(1.0, f / e)) if e > 1e-12 else 0.0
        return t1, t2
    c = _dot(d1, r)
    if e < 1e-12:
        t2 = 0.0
        t1 = max(0.0, min(1.0, -c / a))
        return t1, t2
    b = _dot(d1, d2)
    denom = a * e - b * b
    if denom > 1e-12:
        t1 = max(0.0, min(1.0, (b * f - c * e) / denom))
    else:
        t1 = 0.0
    t2 = (b * t1 + f) / e
    if t2 < 0.0:
        t2 = 0.0
        t1 = max(0.0, min(1.0, -c / a))
    elif t2 > 1.0:
        t2 = 1.0
        t1 = max(0.0, min(1.0, (b - c) / a))
    return t1, t2

def resolve_collisions(state, colliders, body_axis=None, margin=0.0,
                       radial_rbs=None, exclude_rb=None, allowed_collider_rbi=None,
                       hem_weight=None, hem_extra_margin=0.0, accum=None,
                       extra_margin_by_rb=None, margin_cap_by_pair=None):
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
    colliders: [("capsule", p0, p1, radius, group, collision_mask, rb_index),
                ("sphere", center, radius, group, collision_mask, rb_index), ...]
    accum: 指定すると{rb: (dx,dy,dz)}の累積辞書に、このコールで実際に押し出した
        変位ベクトルを加算していく(Noneなら何もしない・既存挙動と完全に同じ)。
        1フレーム内でこの関数は反復拘束解決のため複数回呼ばれることがあるが、
        呼び出しごとの押し出し量は(特に終盤の呼び出しでは)ほぼ収束していて
        小さすぎるため、「弾み」演出のような視覚効果はフレーム全体を通じた
        累積量を見ないと意味のある大きさにならない。この関数自体はもう弾みの
        適用(位置のオーバーシュートやprev操作)を行わない。呼び出し側が
        accumを使って1フレーム分の合計を集計し、フレームの終わりに一度だけ
        (かつ以降の長さ拘束の再適用より後に)効果を適用すること。
    extra_margin_by_rb: {rb_index: 追加クリアランス値} の辞書(rb_size_margin_scale
        から生成)。指定時、該当rbのパーティクルにはそのぶんmarginへ単純加算
        する(hem_weightのような深さ補間はせず定数)。Noneまたは空辞書なら
        加算0で既存挙動と完全に同じ。
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
    # 裾の上乗せ(hem_extra_margin)と剛体実サイズ由来の上乗せ(extra_margin_by_rb)の
    # 両方を、ブロードフェーズのカル半径には保守的に(=最大値を)見積もっておく。
    # 個々のパーティクルに実際に適用される値(_eff_margin)はこれ以下になる。
    _max_extra_by_rb = max(extra_margin_by_rb.values()) if extra_margin_by_rb else 0.0
    _cull_margin = margin + max(hem_extra_margin, 0.0) + _max_extra_by_rb
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
        P0 = P
        px, py, pz = P
        excl = exclude_rb.get(rb) if exclude_rb else None
        _eff_margin = (margin
                       + hem_extra_margin * (hem_weight.get(rb, 0.0) if hem_weight else 0.0)
                       + (extra_margin_by_rb.get(rb, 0.0) if extra_margin_by_rb else 0.0))
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
            # MMDのグループ判定: PMXの剛体はgroup(0-15)と16bitのマスクを持つ。
            # マスクは名前こそ「非衝突グループフラグ」だが、立っているビットが
            # 「そのグループと衝突する」を表す(Bulletのcollision filter mask)。
            # 衝突の条件はBulletと同じく双方向で、相手のmaskに自分のgroupが立ち、
            # かつ自分のmaskに相手のgroupが立っているときだけ衝突する。
            # 【2026-07-28修正】以前はビット=非衝突と解釈して or でスキップして
            # おり、判定が反転していた。実データで確認すると、体のボーン追従剛体は
            # マスク=0xFFFF(全グループと衝突)、揺れ物は自分のgroupだけを落とす、
            # というMMDの定番の作り方になっており、旧解釈では「体のコライダーが
            # 何とも衝突せず、髪同士だけが衝突する」という逆の動作になっていた。
            # 他実装(three.js mmd-parser: groupTarget、saba: m_collisionGroup、
            # blender mmd_tools: import時に反転)も全てこの解釈で一致している。
            # アローリスト方式(allowed_collider_rbi指定)の場合は、この判定自体を
            # 行わない。アローリストに入っているコライダーは無条件に有効な相手と
            # みなす(group/noCollisionMaskの設定がどうであれ関係ない)。
            if allowed_collider_rbi is None:
                if not ((cmask & (1 << p.group)) and (p.collision_mask & (1 << cgroup))):
                    continue
            dvec = _sub(P, C)
            d = _len(dvec)
            # 静止姿勢クランプ: このパーティクルとこのコライダーの組み合わせについて、
            # 静止姿勢(フレーム0)での実クリアランスを超えるマージンは使わない。
            # 超えると静止状態から押し出しが始まり、スカートが傘型に開く。
            _m = _eff_margin
            if margin_cap_by_pair is not None:
                _cap = margin_cap_by_pair.get((rb, rbi))
                if _cap is not None and _m > _cap:
                    _m = _cap
            R = rad + _m
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
        if accum is not None:
            disp = _sub(P, P0)
            if _len(disp) > 1e-12:
                accum[rb] = _add(accum.get(rb, (0.0, 0.0, 0.0)), disp)
        pos[rb] = P


def resolve_collisions_segments(state, chains, colliders, body_axis=None, margin=0.0,
                                radial_rbs=None, exclude_rb=None, allowed_collider_rbi=None,
                                hem_weight=None, hem_extra_margin=0.0, accum=None,
                                extra_margin_by_rb=None, hem_extend=None,
                                margin_cap_by_pair=None, push_preserve_length=True,
                                skip_embedded_parent=True):
    """resolve_collisions と同じ押し出しロジックを使うが、パーティクル単体でなく
    「親→子の区間(セグメント)」をコライダーに対してテストする。

    従来の resolve_collisions は各パーティクルを独立した点として扱うため、
    衝突がたまたま検出された段だけが飛び出し、上下の段には何も伝わらない
    「階段形状」(片足を上げるポーズでの輪切り感)の一因になっていた
    (実機フィードバックで指摘: 「本来ならぶつかった点のベクトルがスカート側の
    点に転写されるべき」)。

    区間として扱うことで、衝突の実際の位置(区間パラメータt、0=親側・1=子側)
    に応じて押し出しの強さを調整する。t=1(衝突点がちょうど子端)のときは
    既存のresolve_collisionsと完全に同じ全量。tが小さい(親寄り)ほど弱め、
    その分は「一段上の区間(この親をさらに子とする区間)」自身のテストが、
    その区間の子端(=この親の位置)側で拾う前提。
    親側は動かさない(実測で親・子の両方を按分して動かす版は、縦チェーンの
    rest_len強制拘束と噛み合わず過剰補正・振動を招いたため撤回した)。

    引数は resolve_collisions とほぼ同じだが、chains(ChainInfoのリスト、
    親子関係を得るために必要)を追加で受け取る。state.part はグループ/
    非衝突マスク判定のみに使う(子側の設定を区間の代表値として使う)。
    """
    if not colliders:
        return
    pos = state.pos
    part = state.part
    bc = bup = None
    if body_axis is not None:
        bc, bup = body_axis

    _bp = []
    _max_extra_by_rb = max(extra_margin_by_rb.values()) if extra_margin_by_rb else 0.0
    _cull_margin = margin + max(hem_extra_margin, 0.0) + _max_extra_by_rb
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
        _bp.append((col, cx, cy, cz, cull, rbi))

    def body_out(P, u_axis):
        if bc is None:
            return None
        rel = _sub(P, bc)
        horiz = _sub(rel, _scale(bup, _dot(rel, bup)))
        if _len(horiz) < 1e-6:
            return None
        n = _norm(horiz)
        if u_axis is not None:
            n = _sub(n, _scale(u_axis, _dot(n, u_axis)))
            if _len(n) < 1e-6:
                return None
            n = _norm(n)
        return n

    # 同じパーティクルが複数の区間・複数のコライダーから補正を受けうるため、
    # 一旦{rb: 補正ベクトルの合計}へ集めてから最後に一括でposへ適用する。
    corrections = {}

    def add_correction(rb, delta):
        if _len(delta) < 1e-12:
            return
        corrections[rb] = _add(corrections.get(rb, (0.0, 0.0, 0.0)), delta)

    for ch in chains:
        plist = ch.particles
        for k in range(1, len(plist)):
            c = plist[k]; pa = plist[k-1]
            if c.kinematic and pa.kinematic:
                continue
            Pa = pos[pa.rb]; Pc = pos[c.rb]
            excl_a = exclude_rb.get(pa.rb) if exclude_rb else None
            excl_c = exclude_rb.get(c.rb) if exclude_rb else None
            _eff_margin_c = (margin
                           + hem_extra_margin * (hem_weight.get(c.rb, 0.0) if hem_weight else 0.0)
                           + (extra_margin_by_rb.get(c.rb, 0.0) if extra_margin_by_rb else 0.0))
            # 裾延長(hem_extend): 終端セグメントのみ、最下段剛体のはみ出し分
            # (半高さ×倍率)だけ子端を仮想延長した線分Pa→Peで衝突評価する。
            # ext=0(hem_extend未指定/該当なし)ならPe==Pcとなり、以降の計算は
            # 既存とビット単位で同一。
            ext = hem_extend.get(c.rb, 0.0) if (hem_extend and k == len(plist) - 1) else 0.0
            if ext > 0.0:
                _sv = _sub(Pc, Pa)
                seg_len = _len(_sv)
                if seg_len > 1e-9:
                    Pe = _add(Pc, _scale(_sv, ext / seg_len))
                else:
                    ext = 0.0
                    Pe = Pc
            else:
                Pe = Pc
            seg_half = _len(_sub(Pe, Pa)) * 0.5
            midx = (Pa[0]+Pe[0])*0.5; midy = (Pa[1]+Pe[1])*0.5; midz = (Pa[2]+Pe[2])*0.5
            for col, ccx, ccy, ccz, cull, rbi in _bp:
                dxc = midx-ccx; dyc = midy-ccy; dzc = midz-ccz
                cull_total = cull + seg_half
                if dxc*dxc+dyc*dyc+dzc*dzc > cull_total*cull_total:
                    continue
                if excl_a is not None and rbi in excl_a and excl_c is not None and rbi in excl_c:
                    continue
                kind = col[0]
                if kind == "capsule":
                    _, a_, b_, rad, cgroup, cmask, _rbi = col
                    t_self, t_col = _closest_segment_segment_t(Pa, Pe, a_, b_)
                    Q = _add(Pa, _scale(_sub(Pe, Pa), t_self))
                    C = _add(a_, _scale(_sub(b_, a_), t_col))
                    u_axis = _norm(_sub(b_, a_))
                elif kind == "sphere":
                    _, C, rad, cgroup, cmask, _rbi = col
                    Q, t_self = _closest_on_segment_t(C, Pa, Pe)
                    u_axis = None
                else:
                    continue
                if allowed_collider_rbi is None:
                    if not ((cmask & (1 << c.group)) and (c.collision_mask & (1 << cgroup))):
                        continue
                # 親端がコライダーの内側にある場合は、区間ではなく子端の点だけで
                # 判定する。モデルによってはチェーンの根元剛体が別のコライダーの
                # 内側に置かれている(IAはスカートの親剛体「下半身」が「下半身1」
                # カプセルの内側 -0.052 にある)。この埋まりは作者が意図した構造で
                # あって解決すべき侵入ではないが、区間判定はこれを深い衝突として
                # 検出し、その一部を毎フレーム子へ転写してしまう(実測: 最上段リング
                # だけが全フレーム押され平均0.0159動かされ、スカートが+0.025開いた
                # まま戻らなくなっていた。押し出された点自体はコライダーから+0.05
                # 離れており、子の位置では説明できなかった)。
                if skip_embedded_parent:
                    if kind == "capsule":
                        _Cpa = _closest_on_segment(Pa, a_, b_)
                    else:
                        _Cpa = C
                    if _len(_sub(Pa, _Cpa)) < rad:
                        if kind == "capsule":
                            C = _closest_on_segment(Pc, a_, b_)
                        Q, t_self = Pc, 1.0
                dvec = _sub(Q, C)
                d = _len(dvec)
                _m_c = _eff_margin_c
                if margin_cap_by_pair is not None:
                    _cap_c = margin_cap_by_pair.get((c.rb, rbi))
                    if _cap_c is not None and _m_c > _cap_c:
                        _m_c = _cap_c
                R = rad + _m_c
                if d >= R:
                    continue
                use_radial = ((bc is not None) and (C[1] < bc[1])
                             and (radial_rbs is None or c.rb in radial_rbs)
                             and (d < R * 0.5))
                n = body_out(Q, u_axis) if use_radial else None
                if n is None:
                    if d > 1e-9:
                        push = _scale(dvec, (R - d) / d)
                    else:
                        push = (R, 0.0, 0.0)
                else:
                    perp = _sub(Q, C)
                    pn = _dot(perp, n)
                    pp = _dot(perp, perp)
                    disc = pn * pn - (pp - R * R)
                    if disc < 0.0:
                        push = _scale(dvec, (R - d) / d) if d > 1e-9 else (0.0, 0.0, 0.0)
                    else:
                        tpush = max(0.0, -pn + math.sqrt(disc))
                        push = _scale(n, tpush)
                if _len(push) < 1e-12:
                    continue
                # 親は動かさず、子側だけをt(0=親端・1=子端)倍で押し出す。
                # 衝突点がちょうど子端(t=1)なら従来のresolve_collisionsと
                # 完全に同じ全量。親寄り(t小)ほど弱め、その分は「一段上の
                # 区間(この親をさらに子とする区間)」自身のテストが、その
                # 区間のt(子端=この親の位置)側で拾う前提(実測で親も同時に
                # 動かす按分は縦拘束と噛み合わず過剰補正になったため撤回)。
                if ext > 0.0:
                    # t_selfは延長線分Pa→Pe上の割合。押し出しの転写は実セグメント
                    # Pa→Pc基準に読み替え、延長部でのヒット(実セグメントの外)は
                    # 「裾の縁が押された」として子端全量(t=1.0)に飽和させる。
                    t_self = min(t_self * (seg_len + ext) / seg_len, 1.0)
                if not c.kinematic and not (excl_c is not None and rbi in excl_c):
                    add_correction(c.rb, _scale(push, t_self))

    # 最終適用: 単純に加算するのではなく、親からの距離がrest_lenちょうどに
    # なるよう再正規化する。これにより「親を中心とした球面上を動く」実質的な
    # 円運動になり、後続の縦チェーン反復でrest_len拘束に無理やり戻される
    # 不自然さ(硬い・ガタガタする見た目)が無くなる(実機フィードバック
    # 「押し出されて移動する点の動きは親を中心にする円運動だと思うが、直線的
    # に動いているように見える」を受けて追加)。
    for rb, delta in corrections.items():
        p_obj = part.get(rb)
        P0 = pos[rb]
        new_pos = _add(P0, delta)
        if p_obj is not None and p_obj.parent != -1 and p_obj.parent in pos and p_obj.rest_len > 1e-9:
            ppos = pos[p_obj.parent]
            rel = _sub(new_pos, ppos)
            rlen = _len(rel)
            if rlen > 1e-9:
                # 半径は「押し出す直前の実際の距離」を保つ。rest_len へ伸ばして
                # しまうと、縮んでいた区間が接触のたびに満長まで伸ばされ、
                # わずかな接触が大きな外向き変位に化ける(実測: IAの腰リングは
                # 全フレームで接触し、押し出し自体は小さいのに平均0.0159動かされ、
                # 開きが+0.025で固定されていた)。長さの管理は後続の縦チェーン
                # 拘束の仕事で、ここは親を中心にした回転(円運動)だけを担う。
                _keep = _len(_sub(P0, ppos)) if push_preserve_length else p_obj.rest_len
                if _keep < 1e-9:
                    _keep = p_obj.rest_len
                new_pos = _add(ppos, _scale(rel, _keep / rlen))
        pos[rb] = new_pos
        if accum is not None:
            actual_disp = _sub(new_pos, P0)
            if _len(actual_disp) > 1e-12:
                accum[rb] = _add(accum.get(rb, (0.0, 0.0, 0.0)), actual_disp)


# ======================================================================
# フェーズA: 適応サブステップ導入のための「トンネリング」検知・計測
# ======================================================================
# 目的: 布(スカート等)がコライダーを素通りする恐れのあるフレームを、
# simulate_step_cloth / resolve_collisions 等の物理計算コードには一切触れずに
# 検出する。フレーム間のキネマティックFK移動距離（アンカー・コライダーとも
# baked から直接計算できる）だけを見て、「1フレームでどれだけ相対移動したか」
# を無次元（コライダー半径比）で報告する。このコードはベイク出力に一切影響
# しない（読み取り専用の計測パス）。
#
# 相対移動量の定義: 「布アンカーの変位」と「コライダーの変位」を別々に求め、
# 三角不等式による安全側（上限）の見積もりとして両者を単純に加算する
# （実際の粒子位置は物理シミュレーションを回さないと分からないため、
# 空間的に厳密な最近傍ペアリングはフェーズAでは行わない。閾値探索の
# 目的には安全側の上限で十分）。
ADAPTIVE_SUBSTEP_MEASURE_VERSION = "2026-07-17a (phase A: detection/measurement only, no physics changes)"


def measure_tunneling(gltf_json, baked, num_frames, physics_gltf, scale,
                      fps=30.0, only_names=None, collision_margin=0.0,
                      log_every=500, min_collider_radius=0.0,
                      collider_name_filter=None):
    """min_collider_radius: この半径未満のコライダー(捻り補助等、視覚的な
    衝突体としての意味が薄い極小剛体)は評価対象から除外する。デフォルト0.0
    は除外なし(全コライダーを見る)。
    collider_name_filter: 剛体名の集合(set)。指定すると、この名前に一致する
    コライダーだけを評価対象にする(例: 脚だけに絞って腕・肩の動きを無視する)。
    デフォルトNoneなら全mode0コライダーを対象にする(既存挙動)。"""
    """フェーズA計測本体。物理シミュレーションは一切呼び出さない。

    戻り値: {
        "frames": [ {frame, max_anchor_disp, max_collider_disp, rel_disp,
                     ratio_radius, ratio_margin, anchor_name, collider_name,
                     collider_radius}, ... ]  （f=1..num_frames-1）
        "n_anchors": int, "n_colliders": int,
    }
    呼び出し側で集計（ヒストグラム・閾値探索）を行う想定。
    """
    nodes = gltf_json["nodes"]
    node_names = [nd.get("name", "?") for nd in nodes]
    print("  [measure] adaptive-substep measurement version: %s"
          % ADAPTIVE_SUBSTEP_MEASURE_VERSION)
    bwm = compute_bone_world_matrices(gltf_json)
    parent = [-1] * len(nodes)
    for i, nd in enumerate(nodes):
        for c in nd.get("children", []):
            parent[c] = i
    chains, parts, excluded, lateral = extract_chains_bfs(
        physics_gltf, bwm, only_names=only_names, bone_parents=parent)

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

    def bone_path(bi):
        path = []
        bb = bi
        while bb != -1:
            path.append(bb); bb = parent[bb]
        path.reverse()
        return path

    _path_cache = {}
    def cached_path(bi):
        p = _path_cache.get(bi)
        if p is None:
            p = bone_path(bi)
            _path_cache[bi] = p
        return p

    def world_at(path, f):
        M = mat_ident()
        for bi in path:
            M = mat_mul(M, local_mat_at(bi, f))
        return M

    def bone_world_pos(bi, f):
        M = world_at(cached_path(bi), f)
        return (M[0][3], M[1][3], M[2][3])

    # アンカー（kinematic）ボーンごとに root→bone のFKチェーンを用意
    # （bake_hair_into_gltf と同一ロジック。物理は一切呼ばない）。
    anchors = {}   # rb -> (bone_index, path)
    for ch in chains:
        p0 = ch.particles[0]
        if p0.kinematic:
            anchors[p0.rb] = (p0.bone, cached_path(p0.bone))

    def anchor_pos_fn(f):
        out = {}
        for rb, (b, path) in anchors.items():
            M = world_at(path, f)
            out[rb] = (M[0][3], M[1][3], M[2][3])
        return out

    _node_names = node_names
    _waist = -1
    for _nm in ("下半身", "腰", "センター", "Center"):
        if _nm in _node_names:
            _waist = _node_names.index(_nm); break

    # 「布」判定: bake_hair_into_gltf と同じ「アンカーが腰以下」基準
    cloth_anchor_rbs = set()
    if _waist >= 0:
        _wy = bone_world_pos(_waist, 0)[1]
        for rb, (b, path) in anchors.items():
            if bone_world_pos(b, 0)[1] <= _wy + 0.05:
                cloth_anchor_rbs.add(rb)
    else:
        cloth_anchor_rbs = set(anchors.keys())

    # コライダー収集（bake_hair_into_gltf と同一ロジック）
    rbs_pg = physics_gltf["rigidBodies"]
    colliders_def = []   # (shape, path, size, lpos, lrot, rb_index, name)
    for _rbi, rb in enumerate(rbs_pg):
        if rb.get("mode") == 0 and rb.get("shape") in (0, 2):
            bi = rb.get("bone", -1)
            if not (0 <= bi < len(nodes)):
                continue
            colliders_def.append((rb["shape"], cached_path(bi), rb["size"],
                                  rb["position"], rb["rotation"],
                                  _rbi, rb.get("name", "?")))
    if collider_name_filter is not None:
        colliders_def = [c for c in colliders_def if c[-1] in collider_name_filter]

    def colliders_at(f):
        cols = []
        for shape, path, size, lpos, lrot, rbi, name in colliders_def:
            W = world_at(path, f)
            M = mat_mul(W, trs_to_mat(lpos, lrot))
            c = (M[0][3], M[1][3], M[2][3])
            if shape == 0:               # 球
                cols.append((rbi, name, c, size[0]))
            else:                        # カプセル（中心=両端の中点、半径のみ使用）
                _, wq = mat_to_trs(M)
                ay = q_rotate_vec(wq, (0.0, 1.0, 0.0))
                half = size[1] * 0.5
                p0 = (c[0]-ay[0]*half, c[1]-ay[1]*half, c[2]-ay[2]*half)
                p1 = (c[0]+ay[0]*half, c[1]+ay[1]*half, c[2]+ay[2]*half)
                center = ((p0[0]+p1[0])*0.5, (p0[1]+p1[1])*0.5, (p0[2]+p1[2])*0.5)
                cols.append((rbi, name, center, size[0]))
        return cols

    # 各布チェーンの「到達距離」(アンカーからどこまで揺れが届きうるか)。
    # スカート等はアンカー(腰側の1粒子目)から下に連なるセグメントの
    # rest_len の合計で近似する。これを使って「そもそも触れうる距離に
    # あるコライダー」だけに絞り込む(でないと、体のどこか別の場所
    # (例: 肘)が速く動いただけで無関係な最大値を拾ってしまう)。
    chain_reach = {}   # anchor_rb -> reach (float)
    for ch in chains:
        p0 = ch.particles[0]
        if p0.kinematic and p0.rb in cloth_anchor_rbs:
            reach = sum(p.rest_len for p in ch.particles[1:])
            chain_reach[p0.rb] = max(chain_reach.get(p0.rb, 0.0), reach)

    _proximity_slack = 0.3 + max(collision_margin, 0.0)  # 保守的な余裕(glTF単位)

    print("  [measure] %d cloth anchor(s) / %d anchor(s) total, %d collider(s), "
          "%d frame(s)" % (len(cloth_anchor_rbs), len(anchors),
                            len(colliders_def), num_frames))

    prev_anchor = {rb: p for rb, p in anchor_pos_fn(0).items()
                   if rb in cloth_anchor_rbs}
    prev_collider = {c[0]: c for c in colliders_at(0)}

    frame_records = []
    for f in range(1, num_frames):
        if f % log_every == 0:
            print("  [measure]   frame %d/%d" % (f, num_frames))
        cur_anchor = {rb: p for rb, p in anchor_pos_fn(f).items()
                      if rb in cloth_anchor_rbs}
        cur_collider = {c[0]: c for c in colliders_at(f)}
        collider_disp = {}   # rbi -> (disp, name, rad, cur_pos)
        for rbi, (rbi2, name, c, rad) in cur_collider.items():
            prev = prev_collider.get(rbi)
            pc = prev[2] if prev else c
            collider_disp[rbi] = (_len(_sub(c, pc)), name, rad, c)

        # フレーム全体のワースト値(全アンカー中の最大)を求める。
        best_rel = 0.0
        best_anchor_disp = 0.0
        best_anchor_name = "?"
        best_collider_disp = 0.0
        best_collider_name = "?"
        best_collider_radius = 0.0

        for rb, p in cur_anchor.items():
            pp = prev_anchor.get(rb, p)
            a_disp = _len(_sub(p, pp))
            b = anchors[rb][0]
            a_name = node_names[b] if 0 <= b < len(node_names) else "?"
            reach = chain_reach.get(rb, 0.0)

            # このアンカー(チェーン)から到達しうる距離+余裕の範囲にある
            # コライダーだけに絞って、その中で最も速く動いたものを探す。
            local_max_disp = 0.0
            local_name = "?"
            local_rad = 0.0
            for rbi, (c_disp, c_name, c_rad, c_pos) in collider_disp.items():
                if c_rad < min_collider_radius:
                    continue
                dist = _len(_sub(p, c_pos))
                if dist > reach + c_rad + _proximity_slack:
                    continue
                if c_disp > local_max_disp:
                    local_max_disp = c_disp
                    local_name = c_name
                    local_rad = c_rad

            rel = a_disp + local_max_disp   # 三角不等式による上限見積もり
            if rel > best_rel:
                best_rel = rel
                best_anchor_disp = a_disp
                best_anchor_name = a_name
                best_collider_disp = local_max_disp
                best_collider_name = local_name
                best_collider_radius = local_rad

        ratio_radius = (best_rel / best_collider_radius) if best_collider_radius > 1e-6 else 0.0
        ratio_margin = (best_rel / collision_margin) if collision_margin > 1e-6 else 0.0

        frame_records.append({
            "frame": f,
            "max_anchor_disp": best_anchor_disp,
            "anchor_name": best_anchor_name,
            "max_collider_disp": best_collider_disp,
            "collider_name": best_collider_name,
            "collider_radius": best_collider_radius,
            "rel_disp": best_rel,
            "ratio_radius": ratio_radius,
            "ratio_margin": ratio_margin,
        })
        prev_anchor = cur_anchor
        prev_collider = cur_collider

    return {
        "frames": frame_records,
        "n_anchors": len(cloth_anchor_rbs),
        "n_colliders": len(colliders_def),
        "num_frames": num_frames,
    }