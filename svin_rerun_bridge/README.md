# SVIn Rerun bridge

`svin_rerun_bridge` converts the ROS 2 visualization streams currently used by
RViz into native Rerun entities. It can stream to a live viewer, write an RRD
recording, or do both at once.

## Build

The package pins the Rerun C++ SDK to `0.40.0`. Its first CMake configure
downloads that SDK and Apache Arrow; later builds reuse the CMake build tree.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ws
colcon build --packages-select svin_rerun_bridge --symlink-install
source install/setup.bash
```

## Live visualization

Enable the bridge in a pinhole-camera launch:

```bash
ros2 launch okvis_ros svin_gopro13_uw.launch.py \
  use_rviz:=false use_rerun:=true
```

The launch starts the native Rerun viewer with its Vulkan renderer by default.
The explicit renderer avoids the EGL initialization failure caused by automatic
renderer selection in the development container.

Use `rerun_native_renderer:=gl` to explicitly select OpenGL. For a browser-based
fallback, use `rerun_viewer_mode:=web` and open `http://127.0.0.1:9090` if the
browser does not open automatically. Use `rerun_start_viewer:=false` when
connecting to a Rerun viewer or server that was started separately.

RViz and Rerun can be enabled at the same time while comparing the views.

### Pinhole cameras

The GoPro and Aqua2 launches use the
`config/svin_rerun_bridge_pinhole.yaml` profile, which is shared by mono and
multi-camera pinhole launches. Each launch overrides the ordered raw image
topic list and `keyframe_image_count` for its calibrated rig:

```bash
ros2 launch okvis_ros svin_gopro13_uw.launch.py \
  use_rviz:=false use_rerun:=true

ros2 bag play \
  /path/to/gopro13_bag \
  --rate 0.5
```

With `camera_model: pinhole`, the bridge reads each pinhole camera's
`T_SC`, image dimensions, focal lengths, and principal point from the active
OKVIS configuration. It logs a semantic Rerun `Pinhole` at
`world/cameras/camN/image`; the original compressed image is logged on that
same entity, so Rerun automatically shows it in 2D and projects it onto the
moving frustum's image plane in 3D. No custom mesh or texture decoding is used.
The pinhole profile also sets `enable_visualization_markers: false`, preventing
the pose graph's manually drawn `/visualization` frustum from appearing on top
of the semantic pinhole frustum.

Rerun's semantic pinhole projection represents the intrinsic K matrix but does
not model OKVIS radial-tangential distortion. The current GoPro profile
therefore displays the original distorted JPEG on a geometrically pinhole
image plane; it does not silently rectify or alter the recorded payload.

The pinhole camera and rendering controls use this schema:

```yaml
camera_model: pinhole
camera_rendering:
  pose_source: pose_graph
  pinhole:
    image_plane_distance: 0.35
    line_width_ui_points: 1.5
```

ROS 2 exposes the nested YAML leaves to the C++ node as dotted parameter names,
for example `camera_rendering.pinhole.image_plane_distance`.

## Live visualization and RRD recording

The recording directory must already exist.

```bash
ros2 launch okvis_ros svin_gopro13_uw.launch.py \
  use_rviz:=false use_rerun:=true \
  rerun_enable_recording:=true \
  rerun_rrd_path:=/absolute/path/to/svin_run.rrd
```

Set `rerun_enable_live:=false` for file-only recording. Keyframe images are
stored as JPEG; adjust `rerun_jpeg_quality` from 1 to 100 (default 85).

Keyframe images are synchronized into one atomic update by exact ROS header
timestamp. By default the bridge waits for `/keyframe_pose` plus two images on
`/keyframe_image_0` and `/keyframe_image_1`, then logs the pose and
`images/keyframe_0` ... `images/keyframe_N` under one `processing_step`. For
more cameras, either provide an explicit list or configure the generated topic
pattern:

```yaml
keyframe_image_topics: [/keyframe_image_0, /keyframe_image_1, /keyframe_image_2]
# or:
keyframe_image_count: 3
keyframe_image_topic_pattern: /keyframe_image_{index}
keyframe_sync_pending_capacity: 32
```

Keyframe images and the optional synchronized pose use reliable ROS QoS with a
configurable KeepLast queue (`keyframe_subscription_depth`, default 32). This
matches the reliable OKVIS publishers and gives the single-threaded bridge
headroom while Rerun serializes a bundle; increase it for unusually long
serialization stalls, subject to image memory use. The bounded pending buffer
drops the oldest incomplete timestamp bundle with a warning. Set
`keyframe_pose_required: false` only for sources that do not
publish `/keyframe_pose`; if a pose is present it is still included in the
bundle.

The moving rig transform is selected explicitly with
`camera_rendering.pose_source`:

- `pose_graph` (default) updates `world/rig` from the last pose of each
  `/loop_closure_path` message. This is the same `T_WC0` source used by
  `CameraPoseVisualization` for its current frustum root, and the path and
  transform are logged in one processing step.
- `keyframe` updates `world/rig` from the synchronized `/keyframe_pose`
  `T_WC0` included in each complete image bundle.
- `odometry` updates `world/rig` from every `/okvis_odometry` message.

Only the selected source writes `world/rig`; the other pose callbacks remain
subscribed but cannot overwrite it. For `pose_graph` and `keyframe`, the rig
root is camera 0 (`C0`) and each camera uses
`T_WCi = T_WC0 * inv(T_SC0) * T_SCi`. For `odometry`, the root is the OKVIS
tracked frame from `publishing_options.trackedBodyFrame`: body-frame odometry
uses `T_WCi = T_WB * T_BS * T_SCi`, while sensor-frame odometry uses
`T_WCi = T_WS * T_SCi`.

## Raw compressed camera payloads

Raw camera transport is enabled with an explicit
`raw_compressed_image_topics` list. Messages are assembled by exact header
timestamp and all cameras in a complete bundle share one `processing_step`.
Their original compressed byte payloads are logged unchanged as `EncodedImage`
on each matching semantic pinhole image entity. If a raw topic has no matching
calibrated camera, it is logged at `images/raw/camN` instead.
`raw_sync_pending_capacity` bounds incomplete timestamps. Raw bytes are never
decoded, modified, or re-encoded.

Point clouds use `point_radius_ui_points` UI points (default `1.5`) for visible
landmarks while preserving per-point colors. During bag replay, complete raw
image/keyframe bundles, paths, and selected odometry poses establish the
trusted sensor clock. PointCloud2 snapshots use the newest trusted timestamp
when available, correcting derived publishers that retain a wall-clock header
while the bag uses ROS time. The bridge does not apply TF to PointCloud2 XYZ;
publishers must already provide coordinates in the logged world frame.

## Timelines

Every ROS callback is logged on two timelines:

- `sensor_time`: the original ROS message timestamp in nanoseconds.
- `processing_step`: a monotonically increasing callback/state-update index.

Selecting `processing_step` reproduces the order in which the live bridge
received visualization updates.

## Current scope

The bridge records matched landmarks and the colored global map below
`world/points`, the OKVIS and loop-closure paths below `world/paths` with
distinct RViz-like colors, synchronized keyframe images/poses, optional raw
compressed camera payloads, and camera-frustum markers below
`world/visualization`.
TF is not bridged yet. Existing pose-graph topics also do not publish the
corrected path and global map atomically at loop closure; a pose-graph revision
event is required for an exact one-step map snap.
