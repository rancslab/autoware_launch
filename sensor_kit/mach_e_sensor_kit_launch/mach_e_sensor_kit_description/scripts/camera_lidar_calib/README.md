# Camera-lidar extrinsic calibration (targetless)

Refines `camera_top/camera_link` in `config/sensor_kit_calibration.yaml` by aligning lidar
depth edges with image edges. No calibration target is needed, but the scene should have
sharp depth edges (poles, building/wall corners, vehicles) at a range of distances.

## 1. Record

Launch Autoware (or sensing) with the car stationary, then:

```bash
ros2 bag record -s mcap -o ~/rosbags/cam_lidar_calib \
  /sensing/camera/camera0/image_raw \
  /sensing/camera/camera0/camera_info \
  /sensing/lidar/top/pointcloud_raw_ex \
  /tf_static
```

20-30 s is enough. `/tf_static` must be included; the scripts start from the calibration
that was active during recording.

## 2. Run

```bash
source /opt/ros/humble/setup.bash
cd <this directory>
python3 extract_pairs.py ~/rosbags/cam_lidar_calib /tmp/calib_data 5   # synced image/cloud pairs
python3 calib_edges.py /tmp/calib_data           # rotation only
python3 calib_edges.py /tmp/calib_data --trans   # rotation + translation
```

`calib_edges.py` prints the refined pose (radians, ready for the YAML) and writes
`result.json` / `result_trans.json` into the data dir.

## 3. Check before committing a result

```bash
# overlay: python3 edge_overlay.py <data_dir> <pair_idx> <out.png> roll pitch yaw(deg) [x y z]
python3 edge_overlay.py /tmp/calib_data 0 /tmp/overlay.png 1.297 1.539 5.599 1.841 0.013 1.679

# compare candidates: "x y z roll_deg pitch_deg yaw_deg"
python3 residual.py /tmp/calib_data "1.851 0 1.711 0 0 0" "1.841 0.013 1.679 1.297 1.539 5.599"
```

Lower median pixel distance is better. Always look at the overlay too: edge alignment can
lock onto a wrong local optimum, and the result can change by about 1 deg with `SIGMA_PX`
in `calib_edges.py`. Translation is weakly observable from a single static scene, so prefer
tape-measured x/y/z unless `--trans` clearly lowers the residual.
