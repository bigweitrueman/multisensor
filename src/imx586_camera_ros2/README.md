# IMX586 Camera Sync

This package publishes synchronized IMX586 camera streams over ROS 2. It
supports the original two-camera setup and an optional four-camera setup, with
standalone `Header` topics for downstream timestamp analysis.

## What it does

- opens two or four V4L2 video nodes
- reads the matching V4L2 subdev controls
- timestamps each image using the trigger time when available
- falls back to the buffer timestamp if no trigger match is found
- adds the current exposure center offset to the timestamp
- publishes image data and headers on separate topics

## Timestamp rule

For each frame, the node:

1. tries to match the frame counter to a trigger event timestamp
2. if that fails, falls back to the V4L2 buffer timestamp
3. if exposure-center correction is enabled, adds `exposure / 2`
4. converts the final monotonic time to ROS time using the sync state file

Exposure values are read from the camera controls and cached at 5 Hz.

## Topics

Default topics:

- `/imx586/cam1/image_raw`
- `/imx586/cam2/image_raw`
- `/imx586/cam1/header`
- `/imx586/cam2/header`
- `/imx586/cam3/image_raw` and `/imx586/cam3/header` when enabled
- `/imx586/cam4/image_raw` and `/imx586/cam4/header` when enabled

## Inputs

Default device paths:

- `trigger_dev`: `/dev/imx586_trigger`
- `trigger_enable`: `/sys/kernel/imx586_trigger/enable`
- `video_dev1`: `/dev/video44`
- `video_dev2`: `/dev/video53`
- `video_dev3`: `/dev/video62`
- `video_dev4`: `/dev/video71`
- `subdev1`: `/dev/v4l-subdev2`
- `subdev2`: `/dev/v4l-subdev7`
- `subdev3`: `/dev/v4l-subdev12`
- `subdev4`: `/dev/v4l-subdev17`
- `enable_cam2`: `true` by default
- `enable_cam3` and `enable_cam4`: enabled by default for the Atlas four-camera setup

The exact device nodes can differ across boards. Override them at launch if
needed.

## Prerequisites

- ROS 2 with `rclcpp`, `sensor_msgs`, and `std_msgs`
- working V4L2 camera driver
- a trigger source for the camera pair
- a sync state file from the radar sync process, usually:
  `/tmp/vanjee_sync_state`

## Build

```bash
cd /path/to/imx586_camera_ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select imx586_camera_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/workspace/install/setup.bash
ros2 launch imx586_camera_ros2 imx586_dual_camera.launch.py
```

If you need a different state file:

```bash
ros2 launch imx586_camera_ros2 imx586_dual_camera.launch.py \
  state_file:=/tmp/vanjee_sync_state
```

For a cam1-only installation with an externally established trigger epoch:

```bash
ros2 launch imx586_camera_ros2 imx586_dual_camera.launch.py \
  enable_cam2:=false \
  control_trigger_enable:=false \
  state_file:=/tmp/vanjee_sync_state
```

For the four-camera Atlas mapping:

```bash
ros2 launch imx586_camera_ros2 imx586_dual_camera.launch.py \
  control_trigger_enable:=false \
  state_file:=/tmp/atlas_joint_sync_state
```

## Typical Workflow

1. Start the radar sync process so the state file is being updated.
2. Start the camera launch file.
3. Verify the image and header topics are present.
4. Record the enabled `/imx586/camN/header` topics and the radar point cloud
   topic if you are checking synchronization.
5. Compare the camera headers against radar timestamps with your offline
   analyzer.

## Launch parameters

The launch file exposes the following inputs:

- `state_file`
- `enable_cam2`
- `control_trigger_enable`
- `image_record_dir`
- `image_record_codec` (`h264` by default, or `raw` for the legacy UYVY stream)
- `image_record_rate_hz` (default `3.0`)
- `image_record_every_n`

The node parameters can be edited in
[`launch/imx586_dual_camera.launch.py`](launch/imx586_dual_camera.launch.py).

Important node parameters:

- `control_trigger_enable`: enable or disable writing the trigger sysfs flag
- `apply_exposure_center_offset`: turn exposure-center correction on or off
- `sensor_vertical_blanking`: default `6192`; applied to every enabled sensor after `STREAMON` for 10 Hz output
- `encoding`: image payload encoding, default `nv12`
- `image_record_codec`: external recording codec. `h264` uses the RK3588 MPP
  hardware encoder and writes one all-intra access unit per selected frame to
  `camN.h264`, with `camN_frames.csv` containing the ROS timestamp and byte
  offset. `raw` retains the legacy `camN.nv12` filename and raw V4L2 payload.
- `video_dev1` ... `video_dev4`: capture devices
- `subdev1` ... `subdev4`: camera control devices

## Notes

- The image payload is published as `nv12` by default.
- External recording defaults to hardware all-intra H.264. This keeps the
  capture thread independent of storage and reduces four 4K streams from
  roughly 288 MB/s raw to about 2-3 MB/s. Use `image_record_codec:=raw` only
  when a raw UYVY dump is specifically required.
- Exposure-center correction is enabled by default.
- The node publishes `Header` messages separately so timestamp analysis can be
  done without decoding the image payload.
- The sync state file maps monotonic time to wall time and is expected to be
  updated by the radar synchronization process.
- Exposure controls are polled at 5 Hz and cached between polls.
