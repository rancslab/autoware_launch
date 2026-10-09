#!/usr/bin/env python3
"""Targetless camera-lidar extrinsic refinement by edge alignment.

Lidar depth-discontinuity points are projected into the image and scored against an
image-edge distance transform. Optimizes camera_top/camera_link pose in sensor_kit_base_link.

usage: calib_edges.py <data_dir> [--trans]   (--trans also refines x/y/z)
"""
import json
import sys

import cv2
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

N_COLS = 2048
SIGMA_PX = 4.0


def mat(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    M[:3, 3] = xyz
    return M


def tf_mat(t):
    M = np.eye(4)
    M[:3, :3] = R.from_quat(t["q"]).as_matrix()
    M[:3, 3] = t["t"]
    return M


def lidar_edges(p, min_range=1.5):
    xyz, ch = p[:, :3], p[:, 4].astype(int)
    r = np.linalg.norm(xyz, axis=1)
    keep = r > min_range
    xyz, ch, r = xyz[keep], ch[keep], r[keep]
    col = (((np.arctan2(xyz[:, 1], xyz[:, 0]) + np.pi) / (2 * np.pi)) * N_COLS).astype(int) % N_COLS
    rng = np.full((128, N_COLS), np.inf)
    idx = np.full((128, N_COLS), -1)
    order = np.argsort(-r)  # nearest written last -> wins
    rng[ch[order], col[order]] = r[order]
    idx[ch[order], col[order]] = order
    edge = np.zeros_like(rng, bool)
    for axis in (0, 1):
        for shift in (1, -1):
            nb = np.roll(rng, shift, axis=axis)
            if axis == 0:  # don't wrap across top/bottom channel
                if shift == 1:
                    nb[0] = np.inf
                else:
                    nb[-1] = np.inf
            valid = np.isfinite(rng)
            # foreground side of a jump (neighbor much farther, or missing = sky/no return)
            with np.errstate(invalid="ignore"):
                jump = valid & (nb - rng > np.maximum(0.5, 0.1 * rng))
            edge |= jump
    sel = idx[edge]
    return xyz[sel[sel >= 0]]


def image_score_map(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.5)
    edges = cv2.Canny(gray, 30, 90)
    dt = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 5)
    return np.exp(-(dt ** 2) / (2 * SIGMA_PX ** 2)).astype(np.float32), edges


def main():
    d = sys.argv[1]
    refine_trans = "--trans" in sys.argv
    m = json.load(open(f"{d}/meta.json"))
    ci = m["camera_info"]
    tf = {(t["parent"], t["child"]): t for t in m["tf_static"]}
    T_kit_os = tf_mat(tf[("sensor_kit_base_link", "os_sensor")])
    T_cam_opt = tf_mat(tf[("camera_top/camera_link", "camera_top/camera_optical_link")])
    t0 = tf[("sensor_kit_base_link", "camera_top/camera_link")]
    xyz0 = np.array(t0["t"])
    rpy0 = R.from_quat(t0["q"]).as_euler("xyz")
    K = np.array(ci["K"]).reshape(3, 3)
    D = np.array(ci["D"])
    W, H = ci["width"], ci["height"]

    frames = []
    for pr in m["pairs"]:
        p = np.load(f"{d}/pc_{pr['idx']}.npy")
        e = lidar_edges(p)
        e_kit = (T_kit_os[:3, :3] @ e.T).T + T_kit_os[:3, 3]
        smap, edges = image_score_map(cv2.imread(f"{d}/img_{pr['idx']}.png"))
        frames.append((e_kit, smap))
        cv2.imwrite(f"{d}/canny_{pr['idx']}.png", edges)
    print(f"{len(frames)} frames, lidar edge points per frame: {[len(f[0]) for f in frames]}")

    def score(x):
        rpy = x[:3]
        xyz = x[3:] if refine_trans else xyz0
        T_opt_kit = np.linalg.inv(mat(xyz, rpy) @ T_cam_opt)
        total, n = 0.0, 0
        for e_kit, smap in frames:
            pc = (T_opt_kit[:3, :3] @ e_kit.T).T + T_opt_kit[:3, 3]
            pc = pc[pc[:, 2] > 0.5]
            uv, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)
            uv = uv.reshape(-1, 2)
            ok = (uv[:, 0] >= 0) & (uv[:, 0] < W - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < H - 1)
            u, v = uv[ok, 0], uv[ok, 1]
            # bilinear lookup
            u0, v0 = u.astype(int), v.astype(int)
            fu, fv = u - u0, v - v0
            s = (smap[v0, u0] * (1 - fu) * (1 - fv) + smap[v0, u0 + 1] * fu * (1 - fv)
                 + smap[v0 + 1, u0] * (1 - fu) * fv + smap[v0 + 1, u0 + 1] * fu * fv)
            total += s.sum()
            n += len(pc)
        return -total / max(n, 1)

    # Coarse grid over rotation (roll/pitch +-4 deg, yaw +-12 deg, 0.5 deg step), then Nelder-Mead.
    deg = np.deg2rad
    best = (score(np.r_[rpy0, xyz0] if refine_trans else rpy0), rpy0)
    print(f"initial score {-best[0]:.4f}  rpy(deg) {np.rad2deg(rpy0).round(3)}")
    grid = np.arange(-4, 4.01, 0.5)
    grid_yaw = np.arange(-12, 12.01, 0.5)
    for dr in grid:
        for dp in grid:
            for dy in grid_yaw:
                rpy = rpy0 + deg(np.array([dr, dp, dy]))
                s = score(np.r_[rpy, xyz0] if refine_trans else rpy)
                if s < best[0]:
                    best = (s, rpy)
    print(f"grid best   {-best[0]:.4f}  rpy(deg) {np.rad2deg(best[1]).round(3)}")
    x0 = np.r_[best[1], xyz0] if refine_trans else best[1]
    step = np.r_[np.full(3, deg(0.3)), np.full(3, 0.05)] if refine_trans else np.full(3, deg(0.3))
    res = minimize(score, x0, method="Nelder-Mead",
                   options={"initial_simplex": np.vstack([x0, x0 + np.diag(step)]),
                            "xatol": 1e-5, "fatol": 1e-6, "maxiter": 4000})
    rpy = res.x[:3]
    xyz = res.x[3:] if refine_trans else xyz0
    print(f"refined     {-res.fun:.4f}  rpy(deg) {np.rad2deg(rpy).round(3)}  xyz {np.round(xyz, 3)}")

    # Sensitivity: score drop for +-0.5 deg on each axis, tells how well each is constrained.
    for i, name in enumerate(("roll", "pitch", "yaw")):
        drops = []
        for sgn in (-1, 1):
            x = res.x.copy()
            x[i] += sgn * deg(0.5)
            drops.append(score(x) - res.fun)
        print(f"  {name}: score drop at +-0.5deg = {np.round(drops, 4)}")

    out = {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2]),
           "roll": float(rpy[0]), "pitch": float(rpy[1]), "yaw": float(rpy[2])}
    json.dump(out, open(f"{d}/result{'_trans' if refine_trans else ''}.json", "w"), indent=1)
    print(json.dumps(out))


if __name__ == "__main__":
    main()
