# -*- coding: utf-8 -*-
"""Quaternion / vector helpers.  Quaternions are (x, y, z, w)."""
import math

QID = (0.0, 0.0, 0.0, 1.0)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qinv(q):
    return (-q[0], -q[1], -q[2], q[3])


def qnorm(q):
    n = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    if n < 1e-12:
        return QID
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def qslerp(a, b, t):
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
    if dot < 0.0:
        b = (-b[0], -b[1], -b[2], -b[3])
        dot = -dot
    if dot > 0.9995:
        return qnorm(tuple(a[i] + t * (b[i] - a[i]) for i in range(4)))
    th = math.acos(max(-1.0, min(1.0, dot)))
    s = math.sin(th)
    wa = math.sin((1 - t) * th) / s
    wb = math.sin(t * th) / s
    return (a[0] * wa + b[0] * wb, a[1] * wa + b[1] * wb,
            a[2] * wa + b[2] * wb, a[3] * wa + b[3] * wb)


def qaxis(axis, angle):
    s = math.sin(angle * 0.5)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle * 0.5))


def q_to_euler_xyz(q):
    """Intrinsic XYZ euler from quaternion."""
    x, y, z, w = q
    # rotation matrix elements
    m20 = 2 * (x * z - w * y)
    sy = -m20
    sy = max(-1.0, min(1.0, sy))
    ey = math.asin(sy)
    if abs(sy) < 0.99999:
        ex = math.atan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
        ez = math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))
    else:
        ex = math.atan2(-2 * (y * z - w * x), 1 - 2 * (x * x + z * z))
        ez = 0.0
    return (ex, ey, ez)


def euler_xyz_to_q(e):
    cx, sx = math.cos(e[0] / 2), math.sin(e[0] / 2)
    cy, sy = math.cos(e[1] / 2), math.sin(e[1] / 2)
    cz, sz = math.cos(e[2] / 2), math.sin(e[2] / 2)
    return qnorm((
        sx * cy * cz + cx * sy * sz,
        cx * sy * cz - sx * cy * sz,
        cx * cy * sz + sx * sy * cz,
        cx * cy * cz - sx * sy * sz,
    ))


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vlen(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def vnorm(v):
    n = vlen(v)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
