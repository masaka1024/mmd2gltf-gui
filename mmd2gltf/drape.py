# -*- coding: utf-8 -*-
"""スカート等の「ドレープの落ち込み」を PMX メッシュから実測する。

背景
----
MMD のスカートボーンは布の外寄り(山側)に置かれていることが多く、実際の
メッシュはそこからさらに内側へ落ち込んでいる。物理ベイクはボーン
(= パーティクル)をコライダーの外へ押し出すので、計算上は貫入していなくても、
内側に落ち込んだ布は脚を突き抜けて見える。これが「本家より貫入が目立つ」の
正体で、rb_size_margin_scale で剛体の厚みを何倍かして埋めていたのは、この
落ち込みを間接的に近似していたことになる。

このモジュールはその落ち込み量をボーン単位で実測し、モデル固有の値として
返す。倍率の手調整を、実測値ベースに置き換えるためのもの。

計測方法
--------
1. 各頂点を「支配ボーン」(最大ウェイトのボーン、既定は 0.5 超)へ割り当てる。
2. ボーンとその子(tail)を結ぶ線分を「布の基準線」とみなす。頂点をこの線分上へ
   投影し、投影点 P を求める(スカートは裾へ向かって広がるので、ボーン位置
   そのものではなく線分上の点を使わないと、広がりぶんを落ち込みと誤認する)。
3. P から見た外向き水平方向 n(体の軸から P へ向かう向き)を求め、
   (P - 頂点)·n を「落ち込み」とする。正なら頂点が基準線より内側にある。
4. ボーンごとに分布のパーセンタイル(既定 p90)を代表値として返す。
   最大値ではなく p90 なのは、少数の極端な頂点で全体を過剰に膨らませない
   ため。本家 MMD 自身も多少の貫入は許容している。

体の軸は、そのボーンから親を辿って最初に見つかる「枝分かれするボーン」
(スカートなら下半身)の XZ 位置を使う。

戻り値の単位は PMX 原単位。glTF 単位で使う場合は unitScale を掛けること。

IA.pmx での実測例(PMX 原単位, p90):
    腰側リング 0.08 → 裾側リング 0.40 (約 5 倍の勾配)
一律のクリアランスでは裾に足りず腰で過剰になる、という実機の手応えと一致する。
"""
import math

__all__ = ["measure_drape_depth", "summarize"]


def _percentile(values, q):
    if not values:
        return 0.0
    s = sorted(values)
    k = (len(s) - 1) * q
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi - k) + s[hi] * (k - lo)


def _children_map(bones):
    ch = [[] for _ in bones]
    for i, b in enumerate(bones):
        p = b.get("parent", -1)
        if 0 <= p < len(bones):
            ch[p].append(i)
    return ch


def _chain_vec(bones, children, i):
    """ボーン i から「次の節」へ向かうベクトル。無ければ None。"""
    b = bones[i]
    p = b["pos"]
    tb = b.get("tail_bone", -1)
    if 0 <= tb < len(bones):
        t = bones[tb]["pos"]
        return (t[0] - p[0], t[1] - p[1], t[2] - p[2])
    off = b.get("tail_offset")
    if off and any(abs(c) > 1e-9 for c in off):
        return (off[0], off[1], off[2])
    kids = children[i]
    if len(kids) == 1:
        t = bones[kids[0]]["pos"]
        return (t[0] - p[0], t[1] - p[1], t[2] - p[2])
    if not kids:
        # 末端ボーン(tail 情報も子も無い)は、親から自分への向きをそのまま延長する。
        # 裾の頂点はこの末端ボーンに支配されていることが多く、ここを落とすと
        # 一番深い落ち込みを取りこぼす。
        par = b.get("parent", -1)
        if 0 <= par < len(bones):
            q = bones[par]["pos"]
            v = (p[0] - q[0], p[1] - q[1], p[2] - q[2])
            if any(abs(c) > 1e-9 for c in v):
                return v
    return None


def _anchor_xz(bones, children, i):
    """体の軸とみなす XZ。親を辿り、最初に枝分かれするボーンの位置を使う。"""
    cur = i
    for _ in range(64):
        p = bones[cur].get("parent", -1)
        if not (0 <= p < len(bones)):
            break
        cur = p
        if len(children[cur]) > 1:
            break
    return bones[cur]["pos"][0], bones[cur]["pos"][2]


def measure_drape_depth(model, percentile=0.9, min_weight=0.5, min_vertices=6):
    """PMX モデルから、ボーンごとのドレープの落ち込みを実測する。

    model      : pmx.parse_pmx() の戻り値
    percentile : 代表値に使う分位(既定 0.9)
    min_weight : 支配ボーンとみなす最小ウェイト
    min_vertices : この数未満しか頂点を持たないボーンは対象外

    戻り値: {ボーン index: 落ち込み量(PMX 原単位, 正のみ)}
    """
    bones = model.get("bones") or []
    verts = model.get("vertices") or []
    if not bones or not verts:
        return {}
    children = _children_map(bones)

    dom = {}
    for v in verts:
        bl, wl = v.get("bones") or (), v.get("weights") or ()
        bi, wmax = -1, 0.0
        for b_, w_ in zip(bl, wl):
            if w_ > wmax:
                bi, wmax = b_, w_
        if wmax >= min_weight and 0 <= bi < len(bones):
            dom.setdefault(bi, []).append(v["pos"])

    out = {}
    for bi, plist in dom.items():
        if len(plist) < min_vertices:
            continue
        vec = _chain_vec(bones, children, bi)
        if vec is None:
            continue
        L = math.sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)
        if L < 1e-6:
            continue
        ux, uy, uz = vec[0] / L, vec[1] / L, vec[2] / L
        bp = bones[bi]["pos"]
        ax, az = _anchor_xz(bones, children, bi)
        depths = []
        for q in plist:
            t = ((q[0] - bp[0]) * ux + (q[1] - bp[1]) * uy + (q[2] - bp[2]) * uz) / L
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            px = bp[0] + vec[0] * t
            pz = bp[2] + vec[2] * t
            nx, nz = px - ax, pz - az
            nl = math.hypot(nx, nz)
            if nl < 1e-6:
                continue
            depths.append(((px - q[0]) * nx + (pz - q[2]) * nz) / nl)
        if len(depths) < min_vertices:
            continue
        d = _percentile(depths, percentile)
        if d > 0.0:
            out[bi] = d
    return out


def summarize(model, depth_by_bone, name_filter=None, limit=None):
    """人が読める一覧を文字列で返す(調査・デバッグ用)。"""
    bones = model.get("bones") or []
    rows = []
    for bi, d in depth_by_bone.items():
        nm = bones[bi]["name"] if 0 <= bi < len(bones) else "?"
        if name_filter and name_filter not in nm:
            continue
        rows.append((nm, bones[bi]["pos"][1], d))
    rows.sort(key=lambda r: -r[1])
    if limit:
        rows = rows[:limit]
    w = max([len(r[0]) for r in rows] or [4])
    return "\n".join("%-*s  Y=%8.3f  depth=%.4f" % (w, n, y, d) for n, y, d in rows)
