# -*- coding: utf-8 -*-
"""VMD (Vocaloid Motion Data 0002) parser.

Reads bone key frames (with the 4 Bezier interpolation curves X/Y/Z/R)
and morph key frames.  Camera / light / shadow sections are skipped.
"""
import struct


def _sjis(b: bytes) -> str:
    return b.split(b"\x00")[0].decode("shift-jis", errors="replace")


def parse_vmd(path):
    with open(path, "rb") as f:
        d = f.read()
    if not d.startswith(b"Vocaloid Motion Data 0002"):
        raise ValueError("Not a VMD 0002 file (old 0001 format is unsupported)")
    o = 30
    model_name = _sjis(d[o:o + 20]); o += 20

    # ---- bone frames -----------------------------------------------------
    (n,) = struct.unpack_from("<I", d, o); o += 4
    bones = {}
    for _ in range(n):
        name = _sjis(d[o:o + 15])
        frame, px, py, pz, rx, ry, rz, rw = struct.unpack_from("<Ifffffff", d, o + 15)
        interp = d[o + 47:o + 111]
        o += 111
        # bezier control points per axis (X,Y,Z,R): x1,y1,x2,y2 in 0..127
        curves = []
        for i in range(4):
            curves.append((interp[i], interp[i + 4], interp[i + 8], interp[i + 12]))
        bones.setdefault(name, []).append({
            "frame": frame,
            "pos": (px, py, pz),
            "rot": (rx, ry, rz, rw),
            "curves": curves,  # [X, Y, Z, R] each (x1, y1, x2, y2)
        })
    for k in bones:
        bones[k].sort(key=lambda f: f["frame"])

    # ---- morph frames ------------------------------------------------------
    morphs = {}
    if o + 4 <= len(d):
        (n,) = struct.unpack_from("<I", d, o); o += 4
        for _ in range(n):
            name = _sjis(d[o:o + 15])
            frame, w = struct.unpack_from("<If", d, o + 15)
            o += 23
            morphs.setdefault(name, []).append((frame, w))
        for k in morphs:
            morphs[k].sort(key=lambda f: f[0])

    # ---- camera / light / self-shadow (skipped) --------------------------
    def skip_section(record_size):
        nonlocal o
        if o + 4 > len(d):
            return
        (cnt,) = struct.unpack_from("<I", d, o); o += 4
        o += cnt * record_size

    skip_section(61)   # camera
    skip_section(28)   # light
    skip_section(9)    # self shadow

    # ---- IK enable frames --------------------------------------------------
    ik_frames = []
    if o + 4 <= len(d):
        (n,) = struct.unpack_from("<I", d, o); o += 4
        for _ in range(n):
            if o + 9 > len(d):
                break
            frame, show, cnt = struct.unpack_from("<IBI", d, o); o += 9
            iks = {}
            for _ in range(cnt):
                if o + 21 > len(d):
                    break
                iks[_sjis(d[o:o + 20])] = bool(d[o + 20])
                o += 21
            ik_frames.append({"frame": frame, "show": bool(show), "ik": iks})
        ik_frames.sort(key=lambda f: f["frame"])

    max_frame = 0
    for keys in bones.values():
        max_frame = max(max_frame, keys[-1]["frame"])
    for keys in morphs.values():
        max_frame = max(max_frame, keys[-1][0])

    return {"model_name": model_name, "bones": bones, "morphs": morphs,
            "ik_frames": ik_frames, "max_frame": max_frame}


def bezier_y(x, x1, y1, x2, y2):
    """Evaluate MMD bezier curve at x (all values normalized to 0..1)."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    # solve t for x by bisection: X(t) = 3(1-t)^2 t x1 + 3(1-t) t^2 x2 + t^3
    lo, hi = 0.0, 1.0
    for _ in range(24):
        t = (lo + hi) * 0.5
        u = 1.0 - t
        xt = 3 * u * u * t * x1 + 3 * u * t * t * x2 + t * t * t
        if xt < x:
            lo = t
        else:
            hi = t
    t = (lo + hi) * 0.5
    u = 1.0 - t
    return 3 * u * u * t * y1 + 3 * u * t * t * y2 + t * t * t


def curve_is_linear(c):
    """A curve is linear when both control points lie on the diagonal."""
    return c[0] == c[1] and c[2] == c[3]
