# BYD K9S bus: lidar bring-up and sensor kit, 2026-10-23

The lab Mach-E PC (rancslab-MIC-770V3W) was moved onto the BYD K9S bus. This report covers getting the bus lidars to stream to it, the first lidar extrinsics, the K9S vehicle dimensions, and the new Autoware packages.

| # | Item | Status |
|---|---|---|
| 1 | Both bus lidars stream to this PC | **Done**. The setting is saved on the lidars but hasn't been checked with a power cycle |
| 2 | Lidar extrinsics | **First estimate**: position tape-measured, rotation estimated from the clouds (±3° yaw) |
| 3 | K9S vehicle dimensions | **Done** except `max_steer_angle` (placeholder) |
| 4 | `byd_k9s_*` sensor kit and vehicle packages | **Done**. Sensing bench test passes at 20 Hz |
| 5 | ADMA on the bus | Open: position not measured, and its data destination is probably 192.168.88.50 |
| 6 | Drive-by-wire / vehicle interface | Open (`vehicle_interface.launch.xml` is empty) |

Use: `vehicle_model:=byd_k9s_vehicle sensor_model:=byd_k9s_sensor_kit`. Bench test: `scripts/run_k9s_sensing.sh`.

---

## 1. Lidars: found and pointed at this PC

**What was on the sensor network (`eno1`)**

| Device | Address | Notes |
|---|---|---|
| Ouster OS1-64 (OS-1-64-U13), SN 122328001026 | 192.168.0.210 | **Front-left**. FW v3.1.0, `1024x20`, `LEGACY` profile |
| Ouster OS1-64 (OS-1-64-U02), SN 122344000555 | 192.168.11.10 | **Front-right**. Same settings |
| GeneSys ADMA | 192.168.88.30 | Same address as on the Mach-E. It ARPs for 192.168.88.50 every 0.5 s |
| Unknown, MAC 00:60:c2:2f:00:89 | 192.168.88.51 | Broadcasts UDP to port 48271 every 2 s; no web page |

The sides were confirmed by a block test: someone stood at one lidar, and only 192.168.11.10's close-range point count rose (1.7k → 4k points within 2.5 m).

**Why nothing arrived:** both lidars had `udp_dest` = `fe80::d31b:ded:256a:bc1e`, the IPv6 link-local address of the bus's previous PC, which isn't on the network. Both also used ports 7502/7503. Both lidars were running and taking PTP time from this PC's `ptp4l` (they don't use it for timestamps: `timestamp_mode` is `TIME_FROM_INTERNAL_OSC`).

**Changes**
- **Lidars** (HTTP API, `POST /api/v1/sensor/config?persist=true`, HTTP 204, and the staged config reads back correctly):
  - 192.168.0.210 sends to 192.168.0.100:7502 (lidar) and :7503 (IMU)
  - 192.168.11.10 sends to 192.168.11.100:7504 (lidar) and :7505 (IMU)
  - FW v3.1.0 has no HTTP reboot command, so this hasn't been checked with a power cycle yet. After the next one, check `ros2 topic hz /sensing/lidar/front_left/pointcloud_raw_ex`.
- **PC:** the NM profile `mach-e-sensors` (on `eno1`) now has 192.168.0.100/24 and 192.168.11.100/24 as well as 192.168.88.100/24 (still needed for the ADMA) and 169.254.6.1/16. This change is in `mach_e_setup/system/setup_network.sh` in the `autoware.mach_e` repo.
- **Nebula:** the Ouster wrapper only listens; it doesn't set `udp_dest`. So the lidar side has to be configured on the lidar.

## 2. Lidar extrinsics

`base_link` = rear axle centre on the ground. `sensor_kit_base_link` = `base_link`.

| Frame | x | y | z | roll | pitch | yaw (rad) |
|---|---|---|---|---|---|---|
| `lidar_front_left` (192.168.0.210) | 7.798 | +1.321 | 1.645 | −3.1388 | +0.0536 | −1.8937 |
| `lidar_front_right` (192.168.11.10) | 7.798 | −1.321 | 1.645 | −3.1072 | +0.0773 | +1.8427 |

**Position** (tape-measured to the lens centre): 64.75 in above the ground, 2 in behind the front face, 52 in either side of the centreline.
- x = wheelbase 222 in + front overhang 87 in − 2 in = 307 in.
- 52 in is 1.2 in outside the 50.8 in half-width of the body, so the lidars sit just outside the body sides.

**Rotation** (`scripts/estimate_lidar_extrinsics.py`):
- **Both lidars are mounted upside down.** In the raw clouds the ground is at z ≈ +1.6 m, so the rotation includes a ~180° roll.
- **Roll/pitch:** from a ground-plane fit (RANSAC + SVD, ~150k inliers, 2 cm rms). The lidars are tilted 3.1° (left) and 4.8° (right) off level. The fitted heights are 1.613 m and 1.627 m, against the tape's 1.645 m.
- **Yaw:** each lidar loses an ~80° sector of ground returns where the bus body is in the way. That sector should be centred on vehicle azimuth 225° (left lidar: behind and to its right) or 135° (right lidar). The left and right results (−108.5°, +105.6°) are nearly mirror images, which is what a symmetric mount should give.
- **Uncertainty:** a second run gave the same right-lidar yaw but a left-lidar yaw 2.5° larger (the blocked sector measured 80° instead of 75°). Treat the yaws as ±3°.

**Checks with both clouds in `base_link`:**
- The ground is at +0.029 m (left) and +0.011 m (right).
- Each lidar sees the side of the bus next to it along y = ±1.29 m, the drawing's half-width.
- Each lidar's blind area is on the opposite side of the bus.

**Still to do:** a proper calibration once the bus can drive, e.g. NDT-based lidar-to-map or lidar-to-lidar ICP on a structured scene. The ground-only parking lot used here constrains yaw only through the body shadow.

## 3. K9S vehicle dimensions

Source: BYD drawing K9S-00001 rev A0 "Exterior design" (dimensions in inches), plus the front track tape-measured.

| Parameter | Value | Source |
|---|---|---|
| `wheel_base` | 5.639 m | 222 in |
| `front_overhang` | 2.210 m | 87 in |
| `rear_overhang` | 3.048 m | 120 in |
| `wheel_tread` | 2.123 m | 83.6 in, front track (measured) |
| `left_overhang` / `right_overhang` | 0.229 m | (101.6 − 83.6) / 2 in, so the width is the 101.6 in body |
| `vehicle_height` | 3.505 m | 138.0 in |
| `wheel_radius` | 0.499 m | 305/70R22.5: 22.5 in / 2 + 0.70 × 305 mm |
| `wheel_width` | 0.305 m | 305/70R22.5 |
| `max_steer_angle` | 0.70 rad | **Placeholder**: needs the front-wheel lock angle |

- **Overall length:** the drawing gives 429.7 in; 120 + 222 + 87 = 429 in. Autoware uses the sum (10.897 m).
- **Dual rear tyres:** don't matter to Autoware. It uses a bicycle model about the rear-axle centre and one `wheel_tread`, and that value is only used for the vehicle's width (footprint, self-crop).
- **Width:** 101.6 in body; 119.5 in over the mirrors. The 80.3 / 80.9 in figures on the rear view are mirror heights, not track widths.
- **`mirror.param.yaml`:** estimated from the drawing: x 7.2–8.4 m, y ±1.52 m, z 1.95–2.75 m.

## 4. Packages

| Package | Contents |
|---|---|
| `sensor_kit/byd_k9s_sensor_kit_launch/byd_k9s_sensor_kit_description` | `sensor_kit_calibration.yaml` (above), URDF with one fixed joint per sensor frame |
| `sensor_kit/byd_k9s_sensor_kit_launch/byd_k9s_sensor_kit_launch` | `lidar.launch.xml` (two `common_sensor_launch/ouster.launch.py` instances under `/sensing/lidar/front_left` and `/sensing/lidar/front_right`), the concat filter, the ADMA IMU/GNSS launch (same as the Mach-E), and `config/ouster.param.yaml` (OS-64, diagnostics for 20 Hz) |
| `vehicle/byd_k9s_vehicle_launch/byd_k9s_vehicle_description` | `vehicle_info.param.yaml`, `mirror.param.yaml`, `simulator_model.param.yaml` (Mach-E copy), a box model of the bus |
| `vehicle/byd_k9s_vehicle_launch/byd_k9s_vehicle_launch` | Empty `vehicle_interface.launch.xml` (no drive-by-wire yet) |

**Notes**
- **Concat filter:** `matching_strategy: naive`. The lidars aren't phase-locked, and Nebula stamps clouds with host arrival time, so the `advanced` timestamp-offset matching doesn't apply. `timeout_sec` is 0.1.
- **Lidar rate:** the lidars still run `1024x20` (20 Hz). Autoware's defaults assume 10 Hz. Switching to `1024x10` halves the network and CPU load; it's a lidar setting plus `ouster.param.yaml` diagnostics (9–11 Hz).
- **URDF comments:** must not contain ": ". `ros2 launch` tries to parse the `robot_description` string as YAML and fails with "Failed to convert ... using yaml rules".

## 5. Bench test

`scripts/run_k9s_sensing.sh`: `autoware.launch.xml` with only the vehicle description and sensing, `fake_vehicle_status.py` for a stopped-vehicle twist, and RViz in `base_link`.

| Topic | Rate |
|---|---|
| `/sensing/lidar/front_left/pointcloud_raw_ex` | 20.0 Hz |
| `/sensing/lidar/front_right/pointcloud_raw_ex` | 20.0 Hz |
| `/sensing/lidar/front_left/pointcloud_before_sync` | 20.0 Hz |
| `/sensing/lidar/front_right/pointcloud_before_sync` | 20.0 Hz |
| `/sensing/lidar/concatenated/pointcloud` | 20.0 Hz |

One concatenated cloud had 41,329 points: exactly 21,055 (left) + 20,274 (right). It was in `base_link`, with 0 points inside the bus box, and the ground at z = +0.025 m.

## 6. Open items

1. **Power cycle:** after the next one, check both lidars still stream (persisted `udp_dest`/ports).
2. **`max_steer_angle`:** measure the front-wheel lock angle.
3. **ADMA:** measure its position (`gnss_link` / `imu_link` are placeholders at `base_link`). Check its data destination: it ARPs for 192.168.88.50, but the driver listens on 192.168.88.100:4444. The bus ADMA config goes in `ADMA_CONFIG=` for `run_mach_e.sh`.
4. **Drive-by-wire:** identify the K9S system and how it connects, then write the vehicle interface.
5. **Lidar extrinsics:** calibrate properly (yaw ±3° now).
6. **Lidar rate:** decide 10 vs 20 Hz.
7. **Map:** a map of the area the bus will drive.
8. **Repo pin:** `repositories/autoware.repos` in `autoware.mach_e` pins `autoware_launch` to a commit; update it to include these packages.
