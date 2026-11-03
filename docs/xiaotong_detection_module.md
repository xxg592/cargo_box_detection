# Cargo-box detection module (Xiaotong, work package 6)

Week 5 deliverable: OpenCV cargo-box detection, detection status and
box-position output, a detection demo, and basic MVP metrics, integrated in
the ROS 2 Jazzy / Gazebo `cargo_box_detector` package.

Everything below was measured on the integrated system, not estimated.

---

## 1. What this module delivers

| Requirement | Delivered as |
| --- | --- |
| OpenCV cargo-box detection code | `cargo_box_detector/detector.py` (pure OpenCV, no ROS) + `cargo_box_detector/detector_node.py` (ROS 2 node) |
| Detection status output | `/cargo_box/detections` — `DETECTED` / `NOT_DETECTED`, `detected_count`, per-colour `color_status`, per-box bounding box |
| Box-position output | `/cargo_box/positions` — per box: `camera_xyz_m`, `base_link_xyz_m`, `odom_xyz_m`, `map_xyz_m` (3D, metres) |
| Detection demo | `src/detect_color_boxes.py` (offline, no ROS) + `src/capture_camera_frame.py` (freeze a live frame) + annotated frames in `results/` and `images/` |
| Basic MVP metrics | `/cargo_box/metrics` + `FINAL METRICS` log line from `metrics_node.py`; recorded run in `results/metrics_latest.json` |
| Live visualisation (bonus) | `/cargo_box/debug_image` — the annotated camera frame for RViz / rqt |

Supporting files: `src/create_test_scenarios.py` (synthetic test images),
`launch/cargo_box_system.launch.py` (one entry point for the whole demo).

---

## 2. Data flow

```
Gazebo RGB camera            Gazebo RGB-D camera
/cargo_robot/front_camera    /cargo_robot/depth_camera/points
        |                             |
        v                             |
 detector_node.py                     |
   detector.py (HSV + contours)       |
        |                             |
        +--> /cargo_box/detections ---+--> fusion_node.py  (TF: camera -> base_link -> odom -> map)
        |         (String/JSON)       |          |
        +--> /cargo_box/debug_image   |          +--> /cargo_box/positions (String/JSON)
                (Image, bgr8)         |
                                      v
                          navigation_node.py -> Nav2 NavigateToPose
        /cargo_box/mission_state  -->  metrics_node.py --> /cargo_box/metrics
```

The detector is deliberately ROS-free: `detector.py` takes a BGR `numpy`
array (or an image path) and returns a result dictionary plus an annotated
image. The ROS node and the offline demo script both call the same function,
so what you test with `src/detect_color_boxes.py` is exactly what runs in
Gazebo.

---

## 3. Interfaces

### 3.1 Inputs

| Topic | Type | Notes |
| --- | --- | --- |
| `/cargo_robot/front_camera` | `sensor_msgs/msg/Image` | 640x480, ~10 Hz, `rgb8` from the Gazebo bridge. Configurable with the `image_topic` parameter. |
| `/cargo_robot/depth_camera/points` | `sensor_msgs/msg/PointCloud2` | Used by `fusion_node.py` only; 320x240 depth at ~10 Hz. |

### 3.2 Outputs

| Topic | Type | Payload |
| --- | --- | --- |
| `/cargo_box/detections` | `std_msgs/msg/String` (JSON) | Detection status and 2D boxes, one message per processed frame |
| `/cargo_box/positions` | `std_msgs/msg/String` (JSON) | 3D box position in four frames, one message per detection frame |
| `/cargo_box/debug_image` | `sensor_msgs/msg/Image` | Annotated frame (`bgr8`); published only while something subscribes |
| `/cargo_box/metrics` | `std_msgs/msg/String` (JSON) | Cumulative metrics, every 2 s and on every mission state change |

### 3.3 `/cargo_box/detections` schema

```json
{
  "image_path": null,
  "image_source": "/cargo_robot/front_camera",
  "image_width": 640,
  "image_height": 480,
  "coordinate_frame": "image_pixels",
  "status": "DETECTED",
  "detected_count": 2,
  "color_status": {"RED": true, "GREEN": true, "BLUE": false, "YELLOW": false},
  "detections": [
    {"color": "RED", "center": [556, 131], "size": [137, 111],
     "bbox": [488, 76, 137, 111], "area": 13739.0}
  ],
  "detection_time_ms": 3.99,
  "frame_id": "cargo_robot/base_link/front_camera"
}
```

* `bbox` is `[x, y, width, height]` in pixels, origin top-left.
* `center` is `[cx, cy]` in pixels — this is the ROI centre that
  `fusion_node.py` reads back when it samples the point cloud.
* Detections are sorted left to right by `x`.
* `status` is `DETECTED` when `detected_count > 0`, otherwise `NOT_DETECTED`.

### 3.4 `/cargo_box/positions` schema

```json
{
  "frame_id": "base_link",
  "navigation_frame_id": "map",
  "source_frame_id": "cargo_robot/base_link/front_depth_camera",
  "cloud_age_s": 0.091,
  "objects": [
    {
      "color": "RED",
      "camera_xyz_m": [3.5809, -1.5244, 0.7105],
      "distance_camera_m": 3.9562,
      "base_link_xyz_m": [3.8707, -1.5244, 0.1449],
      "distance_base_link_m": 4.1625,
      "odom_xyz_m": [3.7068, -1.3398, 0.1449],
      "distance_odom_m": 3.9441,
      "map_xyz_m": [-1.2068, 1.3397, 0.1449],
      "distance_map_m": 1.8089
    }
  ]
}
```

`map_xyz_m` is what the navigation node consumes; the other frames are kept
for debugging. If a transform is unavailable the corresponding field stays
`null` instead of the whole message failing.

### 3.5 Parameters

| Node | Parameter | Default |
| --- | --- | --- |
| `cargo_box_detector_node` | `image_topic` | `/cargo_robot/front_camera` |
| `cargo_box_detector_node` | `debug_image_topic` | `/cargo_box/debug_image` |

---

## 4. Detection method

1. BGR -> HSV conversion.
2. One `cv2.inRange` per colour range, OR-ed together for colours with two
   hue bands (red wraps around 0/180).
3. `MORPH_OPEN` then `MORPH_CLOSE` with a 5x5 kernel to remove speckle and
   fill holes.
4. `cv2.findContours` (`RETR_EXTERNAL`, `CHAIN_APPROX_SIMPLE`).
5. Reject contours below **1000 px²** (about 32x32 px) — this removes
   reflections and texture noise; it is also the main limit on how far away
   a box can be detected (see known issues).
6. Axis-aligned `boundingRect`, sorted left to right, drawn on the annotated
   image.

HSV ranges (OpenCV convention: H 0-179, S/V 0-255):

| Colour | H | S | V |
| --- | --- | --- | --- |
| RED | 0-10 and 170-180 | 100-255 | 80-255 |
| GREEN | 40-85 | 80-255 | 60-255 |
| BLUE | 95-135 | 80-255 | 60-255 |
| YELLOW | 20-35 | 80-255 | 80-255 |

These thresholds were tuned on the flat Gazebo materials used in this world.

---

## 5. Coordinate frames

```
map ──(static, spawn pose)──> odom ──(Gazebo diff-drive)──> base_link ──(static)──> cargo_robot/base_link/front_camera
                                                                     └──(static)──> cargo_robot/base_link/front_depth_camera
```

| Frame | Meaning | Provided by |
| --- | --- | --- |
| `map` | World-aligned warehouse map (1 px = 0.05 m, origin (-5.5, -4.5)) | `map_server` + `static_map_to_odom` |
| `odom` | Odometry origin at the robot spawn pose | Gazebo `DiffDrive` |
| `base_link` | Robot body frame at ground level (see known issue 3) | Gazebo `DiffDrive` |
| `cargo_robot/base_link/front_camera` | RGB camera, `(0.22, 0, 0.16)` m from `base_link`, pitch +0.20 rad | static TF |
| `cargo_robot/base_link/front_depth_camera` | RGB-D camera, same extrinsic | static TF |

The robot spawns at `map (2.5, 0.0)` with yaw 0, so `map -> odom` is the
constant transform `(2.5, 0.0, 0)`. Because Gazebo diff-drive odometry
integrates from the spawn pose, a single static transform is exact and no
AMCL/localisation node is needed for this MVP map.

Image pixel coordinates are converted to metres by sampling the
`PointCloud2` over the middle 50 % of each detection box and taking the
median of the valid points (0.1 m < range <= 5.0 m).

---

## 6. How to run

```bash
# one-time
source /opt/ros/jazzy/setup.bash

# 1. offline detection demo (no ROS, no Gazebo)
cd ~/cargo_box_detection
PYTHONPATH=ros2_ws/src/cargo_box_detector python3 src/detect_color_boxes.py \
    images/test_four_colors.png images/test_empty.png
# -> prints per-colour status, writes results/test_four_colors_detected.png
#    and results/test_four_colors_detection.json

# 2. full integrated demo (Gazebo + Nav2 + detection + pickup/drop-off)
cd ~/cargo_box_detection/ros2_ws
colcon build --symlink-install --packages-select cargo_box_detector
source install/setup.bash
ros2 launch cargo_box_detector cargo_box_system.launch.py spawn_yaw:=3.1416
#    headless:=False shows the Gazebo GUI; start_mission:=False runs Nav2 only

# 3. watch the detection topics in another terminal
source /opt/ros/jazzy/setup.bash
ros2 topic echo /cargo_box/detections
ros2 topic echo /cargo_box/positions
ros2 run rqt_image_view rqt_image_view /cargo_box/debug_image   # annotated stream

# 4. freeze a live frame for the report
python3 src/capture_camera_frame.py --topic /cargo_robot/front_camera \
    --output images/grab.png
```

Reproduce the synthetic test images with `python3 src/create_test_scenarios.py`.

---

## 7. Detection demo record

Measured with the detector as committed (commands in section 6).

| Input | Colour present | Result | Notes |
| --- | --- | --- | --- |
| `images/test_four_colors.png` (synthetic) | RED, GREEN, BLUE, YELLOW | 4/4 detected | boxes 275x395, 275x395, 275x395, 215x395 px |
| `images/test_boxes.png` (synthetic) | RED, GREEN, BLUE | 3/3 detected | 235x335 px each |
| `images/test_empty.png` (synthetic, empty floor) | none | 0 detections | negative control, no false positive |
| `images/gazebo_yellow_view.png` (live Gazebo frame) | YELLOW + GREEN | 2/2 detected | YELLOW bbox 450x383 px at ~1.1 m, GREEN 89x188 px at ~2.9 m |
| `images/cargo_red_blue_view.png` (live Gazebo frame) | RED + BLUE | 2/2 detected | RED 119x120 px at ~3.6 m, BLUE 224x233 px at ~2.1 m, both fully inside the frame |
| `cargo_camera_original.png` (live Gazebo frame) | none in view | 0 detections | robot facing the empty side — no false positive |

Annotated images: `results/*_detected.png`.
Machine-readable results: `results/*_detection.json`.

Live annotated stream: `/cargo_box/debug_image` was captured during a
mission run as `results/demo_live_detection.png` (640x480, `bgr8`).
Verified numerically against a simultaneous raw camera frame: 5553 pixels
differ, of which 2909 are the exact red box outline and 3080 the exact green
outline, while the raw frame contains none of those colours — i.e. the
overlays really are drawn by the node.

---

## 8. Basic MVP metrics

Recorded during one complete autonomous mission (detect -> navigate ->
virtual pickup -> navigate -> virtual drop-off), `results/metrics_latest.json`:

| Metric | Value |
| --- | --- |
| Mission result | `COMPLETED` |
| Mission duration | 32.742 s |
| Detection messages (processed frames) | 326 |
| Frames with at least one detection | 168 |
| Detection rate | 0.5153 (51.5 %) |
| Total box observations | 242 |
| Observations per colour | RED 147, GREEN 20, BLUE 75, YELLOW 0 |
| Detection latency (mean / min / max) | 4.93 / 3.29 / 9.83 ms |
| Mission state transitions | 4 recorded (5 in total: `EXPLORING -> NAVIGATING_TO_BOX` can fire just before the metrics node subscribes) |

Notes on interpretation:

* Detection rate is *not* an accuracy score. The robot spends most of the
  mission driving with boxes out of the 60° horizontal field of view, so
  ~50 % is expected; colour counts show the red box was in view for most of
  the approach.
* Latency is measured per frame inside the node (BGR conversion + detection
  + annotation drawing) on a 640x480 image.
* The mission metrics are only meaningful together with the mission result:
  `FAILED` runs still report detection counts.
* One earlier run aborted because a burst of delayed scans made the costmap
  message filter reject a scan ("timestamp earlier than all the data in the
  transform cache") while the planner was re-planning. Both costmaps now use
  `transform_tolerance: 1.0` and `expected_update_rate: 0.0`, and the run
  above completed with zero `[ERROR]` lines. Under heavy host load the scan
  rate still varies between ~3 and ~10 Hz, so this is worth watching.

---

## 9. Known issues and limits

1. **Distance limit.** The 1000 px² area threshold means a 0.65 m box stops
   being detected at roughly 6-7 m with the 60° FOV camera; in this 10x9 m
   warehouse that is acceptable, but the threshold should become
   distance-aware if the world gets bigger.
2. **Threshold portability.** HSV ranges are tuned on Gazebo's flat
   materials and even lighting. Real cameras (different white balance,
   reflections, shadows) would need re-tuning; a HSV calibration helper is
   the natural next step.
3. **`base_link` z offset.** Gazebo's diff-drive TF publishes `odom ->
   base_link` at the *model* origin, while the model's `base_link` link sits
   0.22 m higher. All reported `z` values (and `base_link`/camera heights)
   are therefore 0.22 m low. Navigation only uses `x`/`y`, so the MVP is not
   affected; the clean fix is to publish `odom -> base_footprint` from
   Gazebo and a static `base_footprint -> base_link` (z = 0.22).
4. **Position is the visible face, not the box centre.** The median of the
   points inside the detection box lies on the surface facing the robot, so
   `map_xyz_m` is offset from the true box centre by roughly half the box
   size along the viewing direction (about 0.3 m for a 0.65 m box). The
   navigation node compensates with a 1.05 m stand-off distance.
5. **No temporal filtering.** Each frame is treated independently; there is
   no tracking or persistence, so a single-frame false positive is published
   as a detection. The FSM only uses the first detection to start a mission,
   which makes the MVP tolerant of this.
6. **Occlusion and partial views.** Two boxes overlapping, or a box half out
   of frame, produce merged or truncated bounding boxes; the point-cloud
   median then mixes both surfaces.
7. **No colour classification ambiguity handling.** YELLOW (H 20-35) and RED
   (H 0-10) can both fire on orange-ish surfaces. The Gazebo palette does not
   contain such colours, so this is untested in simulation.
8. **Metrics colour set.** `metrics_node` counts RED/GREEN/BLUE/YELLOW; any
   other colour label would be added dynamically.

---

## 10. Integration notes for the group

* **Depends on** (provided by other modules): the RGB camera topic and the
  Gazebo world (Mehmet), the RGB-D point cloud and the TF tree
  `map -> odom -> base_link -> camera` (Mohammed / simulation), and
  `/cargo_box/mission_state` from the FSM (Jiayi) for metrics.
* **Provides to others**: `/cargo_box/detections` (2D boxes + colour) and
  `/cargo_box/positions` (`map_xyz_m` per box, ready for goal generation).
* Frames: goals generated from `map_xyz_m` must use `frame_id: map`.
* The module is read-only towards the rest of the system — it never writes
  TF, never commands the robot, and fails soft (malformed input is logged,
  not fatal).

## 11. Git

```bash
git checkout -b feature/xiaotong-detection develop
git add cargo_box_detector/cargo_box_detector/detector.py \
        cargo_box_detector/cargo_box_detector/detector_node.py \
        cargo_box_detector/cargo_box_detector/fusion_node.py \
        cargo_box_detector/cargo_box_detector/metrics_node.py \
        src/ docs/ images/ results/
git commit -m "feat(detection): OpenCV cargo-box detection, 3D position output, demo and MVP metrics"
git push -u origin feature/xiaotong-detection
# then open a merge request into develop
```