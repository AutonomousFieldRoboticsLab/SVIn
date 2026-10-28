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

Enable the bridge in the dual-camera launch:

```bash
ros2 launch okvis_ros svin_insta360X5_dual.launch.py \
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

The GoPro launch uses the separate
`config/svin_rerun_bridge_pinhole.yaml` profile is shared by mono and
multi-camera pinhole launches. Each launch overrides the ordered raw image
topic list and `keyframe_image_count` for its calibrated rig:

```bash
ros2 launch okvis_ros svin_gopro13_uw.launch.py \
  use_rviz:=false use_rerun:=true

ros2 bag play \
  /home/data/Insta360/X5/test/pro_case_uw/mexico26_02/2026_02_28_OtochHa/bag/gopro13 \
  --rate 0.5
```

With `enable_pinhole_camera: true`, the bridge reads each pinhole camera's
`T_SC`, image dimensions, focal lengths, and principal point from the active
OKVIS configuration. It logs a semantic Rerun `Pinhole` at
`world/cameras/camN/image`; the original compressed image is logged on that
same entity, so Rerun automatically shows it in 2D and projects it onto the
moving frustum's image plane in 3D. No custom mesh or texture decoding is used.
The mono profile also sets `enable_visualization_markers: false`, preventing
the pose graph's manually drawn `/visualization` frustum from appearing on top
of the semantic pinhole frustum.

Rerun's semantic pinhole projection represents the intrinsic K matrix but does
not model OKVIS radial-tangential distortion. The current GoPro profile
therefore displays the original distorted JPEG on a geometrically pinhole
image plane; it does not silently rectify or alter the recorded payload.

## Live visualization and RRD recording

The recording directory must already exist.

```bash
ros2 launch okvis_ros svin_insta360X5_dual.launch.py \
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

The moving rig transform is selected explicitly with `mesh_pose_source`:

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
sensor/body frame and each camera uses `T_WCi = T_WR * T_SCi`. The current
dataset has `T_BS = I`, so the odometry expression is valid; a non-identity
body-to-sensor calibration would require an additional transform.

## Double-sphere camera meshes and raw payloads

Set `okvis_config` to an OKVIS calibration containing a `cameras` sequence of
`double_sphere` cameras. With `enable_double_sphere_mesh: true` (the default),
the bridge logs one static, top-left-UV `Mesh3D` per camera at
`world/cameras/camN`, using `double_sphere_depth` metres and a
`double_sphere_grid` × `double_sphere_grid` sampling grid. The static scene
declares Rerun's right-handed, Z-up world convention. Each matching mono/RGB
image can be converted to tightly packed RGB U8, optionally downsampled to
`texture_max_width`, and sent as a `Mesh3D::update_fields()` albedo update.
Whenever the selected pose source updates, the bridge logs a complete temporal
world-from-camera `Transform3D` on the same `world/cameras/camN` entity. The
geometry remains static in the camera optical frame, while the direct camera
transform moves it in world space without relying on a static child transform
under `world/rig`.
Texture decoding is enabled by `enable_mesh_texture_updates` (default true)
and is performed on the first complete bundle and every
`mesh_texture_stride`-th bundle thereafter (default 6). The JPEG copy remains
available at `images/keyframe_N` on every complete bundle regardless of the
mesh texture stride.

`enable_double_sphere_mesh` is the master switch: setting it false disables
static geometry and all texture decoding/updates, while keyframe JPEG and raw
compressed payload logging remain available.

Raw camera transport can be enabled independently with an explicit
`raw_compressed_image_topics` list. Messages are assembled by exact header
timestamp and all cameras in a complete bundle share one `processing_step`.
Their original compressed byte payloads are logged unchanged at
`raw_images/camN` as `EncodedImage`; the decoded color image updates the
matching camera mesh texture only when the texture stride selects that bundle.
`raw_sync_pending_capacity` bounds incomplete timestamps. Raw bytes are never
modified or re-encoded. Raw `EncodedImage` logging remains on every complete
raw bundle even when mesh geometry or texture updates are disabled. When raw
topics are configured, they are authoritative for mesh textures and mono
keyframes do not overwrite them; without raw topics, keyframe images provide
the texture fallback.

Point clouds use `point_radius_ui_points` UI points (default `3.0`) for visible
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
distinct RViz-like colors, synchronized keyframe images/poses, optional camera
meshes and raw compressed payloads, and camera-frustum markers below
`world/visualization`.
TF is not bridged yet. Existing pose-graph topics also do not publish the
corrected path and global map atomically at loop closure; a pose-graph revision
event is required for an exact one-step map snap.
