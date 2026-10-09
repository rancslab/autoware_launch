#!/usr/bin/env python3
"""Extract time-synced camera/lidar pairs from a calibration bag.

usage: extract_pairs.py <bag_dir> <out_dir> [n_pairs]
"""
import json
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

IMG = "/sensing/camera/camera0/image_raw"
INFO = "/sensing/camera/camera0/camera_info"
PC = "/sensing/lidar/top/pointcloud_raw_ex"
TF = "/tf_static"


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def image_to_bgr(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return img if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc in ("mono8", "8uc1"):
        return cv2.cvtColor(buf.reshape(msg.height, msg.step)[:, : msg.width], cv2.COLOR_GRAY2BGR)
    if enc in ("yuv422", "uyvy", "yuv422_yuy2", "yuyv"):
        img = buf.reshape(msg.height, msg.width, 2)
        code = cv2.COLOR_YUV2BGR_UYVY if enc in ("yuv422", "uyvy") else cv2.COLOR_YUV2BGR_YUYV
        return cv2.cvtColor(img, code)
    raise RuntimeError(f"unsupported encoding {msg.encoding}")


def main():
    bag, out = sys.argv[1], sys.argv[2]
    n_pairs = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    os.makedirs(out, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # Pass 1: header stamps only for clouds/images, keep info + tf.
    img_t, pc_t, info, tfs = [], [], None, []
    reader.set_filter(rosbag2_py.StorageFilter(topics=[IMG, PC, INFO, TF]))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == IMG:
            img_t.append(stamp(msg))
        elif topic == PC:
            pc_t.append(stamp(msg))
        elif topic == INFO and info is None:
            info = msg
        elif topic == TF:
            for t in msg.transforms:
                tr, q = t.transform.translation, t.transform.rotation
                tfs.append({"parent": t.header.frame_id, "child": t.child_frame_id,
                            "t": [tr.x, tr.y, tr.z], "q": [q.x, q.y, q.z, q.w]})
    img_t, pc_t = np.array(img_t), np.array(pc_t)

    # Pick clouds spread over the bag, each with its nearest image.
    picks = np.linspace(len(pc_t) * 0.1, len(pc_t) * 0.9, n_pairs).astype(int)
    want_pc = {round(pc_t[i], 6) for i in picks}
    want_img = {}
    for i in picks:
        j = int(np.argmin(np.abs(img_t - pc_t[i])))
        want_img[round(img_t[j], 6)] = (round(pc_t[i], 6), img_t[j] - pc_t[i])

    pairs = {}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[IMG, PC]))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        s = round(stamp(msg), 6)
        if topic == PC and s in want_pc:
            fields = [f.name for f in msg.fields]
            names = ["x", "y", "z"] + [n for n in ("intensity", "channel", "azimuth") if n in fields]
            rec = point_cloud2.read_points(msg, field_names=names, skip_nans=True)
            pts = np.stack([rec[n].astype(np.float32) for n in names], axis=1)
            pairs.setdefault(s, {})["pc"] = pts
            pairs[s]["pc_frame"] = msg.header.frame_id
            pairs[s]["pc_fields"] = fields
        elif topic == IMG and s in want_img:
            key, dt = want_img[s]
            pairs.setdefault(key, {})["img"] = image_to_bgr(msg)
            pairs[key]["img_frame"] = msg.header.frame_id
            pairs[key]["encoding"] = msg.encoding
            pairs[key]["dt"] = float(dt)

    meta = {"camera_info": {"width": info.width, "height": info.height,
                            "frame_id": info.header.frame_id,
                            "K": list(info.k), "D": list(info.d),
                            "distortion_model": info.distortion_model},
            "tf_static": tfs, "pairs": []}
    for k, (s, p) in enumerate(sorted(pairs.items())):
        if "pc" not in p or "img" not in p:
            continue
        cv2.imwrite(os.path.join(out, f"img_{k}.png"), p["img"])
        np.save(os.path.join(out, f"pc_{k}.npy"), p["pc"])
        meta["pairs"].append({"idx": k, "stamp": s, "dt_img_minus_pc": p["dt"],
                              "n_points": int(len(p["pc"])), "pc_frame": p["pc_frame"],
                              "pc_fields": p["pc_fields"], "img_frame": p["img_frame"],
                              "encoding": p["encoding"], "img_shape": list(p["img"].shape)})
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps({k: v for k, v in meta.items() if k != "tf_static"}, indent=1))
    print(f"{len(tfs)} static transforms")


if __name__ == "__main__":
    main()
