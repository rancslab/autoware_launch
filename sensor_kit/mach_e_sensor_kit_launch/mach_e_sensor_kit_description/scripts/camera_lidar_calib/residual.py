#!/usr/bin/env python3
"""Compare candidate camera_link poses by lidar-edge to image-edge pixel distance.

usage: residual.py <data_dir> "x y z roll_deg pitch_deg yaw_deg" ["..." ...]
Run calib_edges.py on the data dir first (it writes the canny_<idx>.png images used here).
"""
import json
import sys

import cv2
import numpy as np

from calib_edges import lidar_edges, mat, tf_mat


def main():
    d = sys.argv[1]
    m = json.load(open(f"{d}/meta.json"))
    ci = m["camera_info"]
    tf = {(t["parent"], t["child"]): t for t in m["tf_static"]}
    K = np.array(ci["K"]).reshape(3, 3)
    D = np.array(ci["D"])
    T_kit_os = tf_mat(tf[("sensor_kit_base_link", "os_sensor")])
    T_cam_opt = tf_mat(tf[("camera_top/camera_link", "camera_top/camera_optical_link")])

    data = []
    for pr in m["pairs"]:
        e = lidar_edges(np.load(f"{d}/pc_{pr['idx']}.npy"))
        e = (T_kit_os[:3, :3] @ e.T).T + T_kit_os[:3, 3]
        dt = cv2.distanceTransform(255 - cv2.imread(f"{d}/canny_{pr['idx']}.png", 0), cv2.DIST_L2, 5)
        data.append((e, dt))

    for cand in sys.argv[2:]:
        v = [float(a) for a in cand.split()]
        T = np.linalg.inv(mat(v[:3], np.deg2rad(v[3:6])) @ T_cam_opt)
        dists = []
        for e, dt in data:
            pc = (T[:3, :3] @ e.T).T + T[:3, 3]
            pc = pc[pc[:, 2] > 0.5]
            uv = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2).astype(int)
            ok = ((uv[:, 0] >= 0) & (uv[:, 0] < dt.shape[1]) & (uv[:, 1] >= 0)
                  & (uv[:, 1] < dt.shape[0]))
            dists.append(dt[uv[ok, 1], uv[ok, 0]])
        r = np.concatenate(dists)
        print(f"[{cand}]  median {np.median(r):5.2f}px  within3px {(r < 3).mean() * 100:5.1f}%  "
              f"mean(<15px) {r[r < 15].mean():4.2f}")


if __name__ == "__main__":
    main()
