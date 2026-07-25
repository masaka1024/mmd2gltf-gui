# -*- coding: utf-8 -*-
"""Bake a VMD motion onto a PMX skeleton, MMD-style.

Everything here runs in MMD's native left-handed coordinate system.
Pipeline per frame (30 fps, VMD native):
  1. evaluate VMD tracks (Bezier interpolation per X/Y/Z/R channel)
  2. process bones in MMD deform order (afterPhysics, layer, index):
     - apply inherit rotation / translation (付与)
     - solve IK chains by CCD with per-link angle limits
  3. record the resulting local translation / rotation of every bone.

Physics simulation is NOT performed; physics-driven bones keep the pose
their animation / parents give them.
"""
import math

from .mathutil import (QID, qmul, qinv, qnorm, qrot, qslerp, qaxis,
                       q_to_euler_xyz, euler_xyz_to_q,
                       vadd, vsub, vlen, vnorm, vcross, vdot)
from .vmd import bezier_y, curve_is_linear

FPS = 30.0


class Track:
    """Per-bone VMD key list with MMD Bezier interpolation."""

    def __init__(self, keys):
        self.keys = keys
        self.frames = [k["frame"] for k in keys]

    def sample(self, frame):
        keys, frames = self.keys, self.frames
        if frame <= frames[0]:
            k = keys[0]
            return k["pos"], k["rot"]
        if frame >= frames[-1]:
            k = keys[-1]
            return k["pos"], k["rot"]
        # binary search
        lo, hi = 0, len(frames) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if frames[mid] <= frame:
                lo = mid
            else:
                hi = mid
        k0, k1 = keys[lo], keys[hi]
        span = k1["frame"] - k0["frame"]
        x = (frame - k0["frame"]) / span if span else 0.0
        pos = []
        for axis in range(3):
            c = k1["curves"][axis]
            if curve_is_linear(c):
                t = x
            else:
                t = bezier_y(x, c[0] / 127.0, c[1] / 127.0,
                             c[2] / 127.0, c[3] / 127.0)
            pos.append(k0["pos"][axis] + (k1["pos"][axis] - k0["pos"][axis]) * t)
        c = k1["curves"][3]
        if curve_is_linear(c):
            t = x
        else:
            t = bezier_y(x, c[0] / 127.0, c[1] / 127.0,
                         c[2] / 127.0, c[3] / 127.0)
        rot = qslerp(qnorm(k0["rot"]), qnorm(k1["rot"]), t)
        return tuple(pos), rot


class Skeleton:
    def __init__(self, model):
        self.bones = model["bones"]
        n = len(self.bones)
        self.rest = []            # local rest offset relative to parent
        for i, b in enumerate(self.bones):
            p = b["parent"]
            if 0 <= p < n:
                pp = self.bones[p]["pos"]
                self.rest.append(vsub(b["pos"], pp))
            else:
                self.rest.append(tuple(b["pos"]))
        # deform order: (after-physics, layer, index)
        self.order = sorted(
            range(n),
            key=lambda i: (1 if self.bones[i]["flags"] & 0x1000 else 0,
                           self.bones[i]["layer"], i))
        self.local_q = [QID] * n
        self.local_t = [(0.0, 0.0, 0.0)] * n   # animation offset (adds to rest)
        self._world = {}

    def reset(self):
        n = len(self.bones)
        self.local_q = [QID] * n
        self.local_t = [(0.0, 0.0, 0.0)] * n
        self._world = {}

    def invalidate(self):
        self._world = {}

    def world(self, i):
        """Return (worldQ, worldPos) of bone i."""
        w = self._world.get(i)
        if w is not None:
            return w
        b = self.bones[i]
        p = b["parent"]
        lt = vadd(self.rest[i], self.local_t[i])
        if 0 <= p < len(self.bones):
            pq, pp = self.world(p)
            wq = qmul(pq, self.local_q[i])
            wp = vadd(pp, qrot(pq, lt))
        else:
            wq = self.local_q[i]
            wp = lt
        self._world[i] = (wq, wp)
        return (wq, wp)

    # ------------------------------------------------------------------
    def pose(self, sampled, ik_enabled=None):
        """sampled: dict bone_index -> (pos_offset, quat). Runs full pipeline.

        ik_enabled: optional set of IK bone indices allowed to solve;
        None means all IK bones are enabled.
        """
        self.reset()
        for i, (pos, rot) in sampled.items():
            self.local_t[i] = pos
            self.local_q[i] = qnorm(rot)

        for i in self.order:
            b = self.bones[i]
            fl = b["flags"]
            if fl & (0x0100 | 0x0200):
                src = b.get("inherit_parent", -1)
                ratio = b.get("inherit_ratio", 1.0)
                if 0 <= src < len(self.bones):
                    if fl & 0x0100:  # inherit rotation
                        add = qslerp(QID, self.local_q[src], ratio)
                        self.local_q[i] = qnorm(qmul(add, self.local_q[i]))
                    if fl & 0x0200:  # inherit translation
                        st = self.local_t[src]
                        self.local_t[i] = vadd(
                            self.local_t[i],
                            (st[0] * ratio, st[1] * ratio, st[2] * ratio))
                    self.invalidate()
            if fl & 0x0020 and "ik" in b:
                if ik_enabled is None or i in ik_enabled:
                    self._solve_ik(i, b["ik"])

    # ------------------------------------------------------------------
    def _solve_ik(self, ik_bone, ik):
        eff = ik["target"]
        if not (0 <= eff < len(self.bones)):
            return
        goal = self.world(ik_bone)[1]
        limit = ik["limit_angle"]
        for _ in range(max(1, ik["loop"])):
            done = False
            for li, link in enumerate(ik["links"]):
                j = link["bone"]
                lq, lp = self.world(j)
                eff_pos = self.world(eff)[1]
                if vlen(vsub(eff_pos, goal)) < 1e-4:
                    done = True
                    break
                inv = qinv(lq)
                v1 = vnorm(qrot(inv, vsub(eff_pos, lp)))
                v2 = vnorm(qrot(inv, vsub(goal, lp)))
                d = max(-1.0, min(1.0, vdot(v1, v2)))
                ang = math.acos(d)
                if ang < 1e-6:
                    continue
                ang = min(ang, limit * (li + 1))
                axis = vnorm(vcross(v1, v2))
                if vlen(axis) < 1e-6:
                    continue
                q = qaxis(axis, ang)
                self.local_q[j] = qnorm(qmul(self.local_q[j], q))
                if "min" in link:
                    e = list(q_to_euler_xyz(self.local_q[j]))
                    for c in range(3):
                        e[c] = max(link["min"][c], min(link["max"][c], e[c]))
                    self.local_q[j] = euler_xyz_to_q(e)
                self.invalidate()
            if done:
                break


def bake(model, vmd, solve_ik=True, step=1, progress=None,
         disable_ik=None, use_vmd_ik_frames=True):
    """Sample the motion every `step` VMD frames and run the deform pipeline.

    disable_ik: list of substrings; IK bones whose name contains one of them
        are never solved (e.g. ["足"] disables leg/toe IK).
    use_vmd_ik_frames: honour IK on/off key frames stored in the VMD.

    Returns (times, bone_data) where bone_data maps bone index to
    {"t": [vec3 local translation incl. rest] or None, "r": [quat] or None}
    (channels that never deviate from rest are dropped).
    """
    bones = model["bones"]

    # map truncated shift-jis names (VMD stores max 15 bytes) to bone index
    name_map = {}
    for i, b in enumerate(bones):
        try:
            key = b["name"].encode("shift-jis", errors="replace")[:15]
        except Exception:
            key = b["name"].encode("utf-8", errors="replace")[:15]
        name_map.setdefault(key, i)

    tracks = {}
    unmatched = []
    for name, keys in vmd["bones"].items():
        key = name.encode("shift-jis", errors="replace")[:15]
        i = name_map.get(key)
        if i is None:
            unmatched.append(name)
            continue
        tracks[i] = Track(keys)

    sk = Skeleton(model)

    # --- which IK bones may solve -----------------------------------------
    all_ik = [i for i, b in enumerate(bones)
              if b["flags"] & 0x0020 and "ik" in b]
    base_enabled = set(all_ik)
    if not solve_ik:
        base_enabled = set()
    if disable_ik:
        for i in all_ik:
            name = bones[i]["name"]
            if any(pat in name for pat in disable_ik):
                base_enabled.discard(i)

    # VMD IK on/off key frames (name -> bone index, truncated to 20 bytes)
    ik_key_frames = []
    if use_vmd_ik_frames and vmd.get("ik_frames"):
        ik_name_map = {}
        for i in all_ik:
            try:
                k = bones[i]["name"].encode("shift-jis", errors="replace")[:20]
            except Exception:
                k = bones[i]["name"].encode("utf-8", errors="replace")[:20]
            ik_name_map.setdefault(k, i)
        for fr in vmd["ik_frames"]:
            m = {}
            for name, on in fr["ik"].items():
                i = ik_name_map.get(
                    name.encode("shift-jis", errors="replace")[:20])
                if i is not None:
                    m[i] = on
            ik_key_frames.append((fr["frame"], m))

    def enabled_at(frame):
        if not ik_key_frames:
            return base_enabled
        state = {}
        for f0, m in ik_key_frames:
            if f0 > frame:
                break
            state.update(m)
        return {i for i in base_enabled if state.get(i, True)}

    last = vmd["max_frame"]
    frames = list(range(0, last + 1, step))
    if frames[-1] != last:
        frames.append(last)

    n = len(bones)
    times = [f / FPS for f in frames]
    rots = [[] for _ in range(n)]
    trans = [[] for _ in range(n)]

    for fi, f in enumerate(frames):
        sampled = {i: tr.sample(f) for i, tr in tracks.items()}
        sk.pose(sampled, ik_enabled=enabled_at(f))
        for i in range(n):
            rots[i].append(sk.local_q[i])
            trans[i].append(vadd(sk.rest[i], sk.local_t[i]))
        if progress and fi % 200 == 0:
            progress(fi, len(frames))

    out = {}
    for i in range(n):
        anim_r = any(abs(q[0]) + abs(q[1]) + abs(q[2]) > 1e-7 or q[3] < 0.999999
                     for q in rots[i])
        rest = sk.rest[i]
        anim_t = any(vlen(vsub(t, rest)) > 1e-6 for t in trans[i])
        if anim_r or anim_t:
            out[i] = {"r": rots[i] if anim_r else None,
                      "t": trans[i] if anim_t else None}
    return times, out, unmatched
