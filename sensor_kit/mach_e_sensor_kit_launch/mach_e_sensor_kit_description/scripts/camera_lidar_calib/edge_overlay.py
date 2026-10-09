"""usage: edge_overlay.py data_dir idx out.png roll pitch yaw(deg) [x y z]"""
import sys, json, numpy as np, cv2
sys.path.insert(0, sys.path[0])
from calib_edges import lidar_edges, mat, tf_mat
d, idx, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
rpy = np.deg2rad([float(a) for a in sys.argv[4:7]])
m = json.load(open(f"{d}/meta.json")); ci = m["camera_info"]
tf = {(t["parent"], t["child"]): t for t in m["tf_static"]}
xyz = [float(a) for a in sys.argv[7:10]] if len(sys.argv) > 9 else tf[("sensor_kit_base_link", "camera_top/camera_link")]["t"]
T = np.linalg.inv(mat(xyz, rpy) @ tf_mat(tf[("camera_top/camera_link", "camera_top/camera_optical_link")])) @ tf_mat(tf[("sensor_kit_base_link", "os_sensor")])
e = lidar_edges(np.load(f"{d}/pc_{idx}.npy"))
pc = (T[:3, :3] @ e.T).T + T[:3, 3]; pc = pc[pc[:, 2] > 0.5]
uv, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), np.array(ci["K"]).reshape(3, 3), np.array(ci["D"])); uv = uv.reshape(-1, 2)
img = cv2.imread(f"{d}/img_{idx}.png"); img = (img * 0.55).astype(np.uint8)
canny = cv2.imread(f"{d}/canny_{idx}.png", 0); img[canny > 0] = (255, 255, 255)
z = pc[:, 2]
col = cv2.applyColorMap((np.clip((z - 1) / 12, 0, 1) * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
for (u, v), c in zip(uv, col):
    if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]: cv2.circle(img, (int(u), int(v)), 2, tuple(int(x) for x in c), -1)
cv2.putText(img, f"rpy deg {sys.argv[4:7]}", (20, 40), 0, 1.1, (0, 255, 255), 2)
cv2.imwrite(out, img)
