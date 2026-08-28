# rk3588s 多传感器同步采集

本仓库保存 rk3588s RK3588 板卡上 IMX586 四路相机与万集 WLR-722 雷达的核心同步、采集和录制代码。仓库仅包含源码与配置，不包含采集数据、ROS 2 构建目录、预编译内核模块或厂商 SDK 全量源码。

## 核心数据流

```text
imx586_trigger_sync
  ├─ GPIO54：10 Hz 相机触发
  └─ GPIO142：1 Hz 雷达 PPS
             ↓
UART7 伪 RMC + 同一触发 epoch
             ↓
/tmp/rk3588s_joint_sync_state
             ↓
四路 IMX586：触发计数匹配 + 曝光中心修正
  ├─ ROS Image/Header（约 10 Hz）
  └─ H.264 + frames.csv（默认 3 Hz）

WLR-722 点云/IMU + 四路 Header → MCAP rosbag
```

注意：当前实现由板端向雷达输出 PPS，并通过 `/dev/ttyS7` 向雷达发送伪 RMC；不是从雷达读取 PPS/RMC。

## 目录

- `kernel/imx586_trigger_sync/`：GPIO54/GPIO142 同时序触发内核模块源码。
- `sync/vanjee_pseudo_utc_sync.py`：绑定触发 epoch、生成伪 RMC、写同步状态文件。
- `src/imx586_camera_ros2/`：四路 V4L2 采集、时间戳修正、ROS 发布和 MPP H.264 落盘。
- `tools/load_rk3588s_trigger_sync.sh`：当前板卡的模块加载参数。
- `tools/record_rk3588s_session.sh`：只录雷达、IMU、相机 Header 和 `/tf_static`。
- `tools/rk3588s_sync_check.py`：在线同步检查。
- `tools/analyze_vanjee_frames_ros2.py`：雷达帧时间与有效点分析。
- `config/`：rosbag QoS 与 WLR-722 运行配置。

## 当前硬件与时序参数

| 功能 | 当前映射/参数 |
| --- | --- |
| 相机触发 | GPIO54，10 Hz，高电平 5 ms |
| 雷达 PPS 输入信号源 | 板端 GPIO142 / GPIO4_B6，1 Hz，高电平 10 ms |
| PPS 相对相机触发相位 | `5,689,000 ns` |
| 雷达 RMC | UART7，`/dev/ttyS7`，9600 baud |
| 雷达网络 | 板端 `eth1: 192.168.2.88/24`，雷达 `192.168.2.86` |
| 雷达供电 | `/sys/class/leds/lidar_12v_en/brightness` |
| ROS 2 | Jazzy，`ROS_DOMAIN_ID=37` |

GPIO 输出为 1.8 V，而雷达 PPS 接口标称为 3.3 V。量产设计必须增加 1.8 V 到 3.3 V 电平转换，禁止将 3.3 V 直接反灌 rk3588s GPIO。

源码中保留了一些通用/历史默认值，部署时以 `tools/load_rk3588s_trigger_sync.sh` 的 `PPS_GPIO_PIN` 和 `PPS_PHASE_NS` 为准；同步程序的 `--imx586-pps-phase-ns` 必须使用同一个相位值。

## 构建

安装 ROS 2 Jazzy、V4L2、Rockchip MPP 开发库以及万集 `vanjee_lidar_sdk`。厂商 SDK 未完整收录；`config/vanjee_lidar_sdk/config_722.yaml` 依赖 SDK 安装目录中的 WLR-722 标定 CSV。

```bash
cd /userdata/rk3588s
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select imx586_camera_ros2 vanjee_lidar_sdk \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

内核模块必须针对板卡正在运行的内核构建：

```bash
cd /userdata/rk3588s/kernel/imx586_trigger_sync
make
modinfo ./imx586_trigger_sync.ko
uname -r
```

确认 `vermagic` 匹配后，把生成的模块放到加载脚本配置的路径。`.ko` 不进入 Git。

## 启动顺序

以下命令假设仓库部署在 `/userdata/rk3588s`。

1. 打开雷达电源、配置网络，并停止会占用雷达资源的渲染服务：

   ```bash
   echo 1 > /sys/class/leds/lidar_12v_en/brightness
   ip link set eth1 up
   ip addr replace 192.168.2.88/24 dev eth1
   systemctl stop pointcloud-renderer.service
   ```

2. 加载共同触发模块：

   ```bash
   PPS_PHASE_NS=5689000 /userdata/rk3588s/tools/load_rk3588s_trigger_sync.sh
   ```

3. 启动 WLR-722 ROS 2 驱动：

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /userdata/rk3588s/install/setup.bash
   export ROS_DOMAIN_ID=37 ROS_LOCALHOST_ONLY=0
   ros2 run vanjee_lidar_sdk vanjee_lidar_sdk_node --ros-args \
     -p config_path:=/userdata/rk3588s/install/vanjee_lidar_sdk/share/vanjee_lidar_sdk/config/config_722.yaml
   ```

4. 绑定共同 epoch，并向雷达发送伪 RMC：

   ```bash
   PPS_PHASE_NS=5689000
   START_TIME="$(date '+%Y-%m-%dT%H:%M:%S')"
   python3 /userdata/rk3588s/sync/vanjee_pseudo_utc_sync.py \
     --skip-pwm --serial /dev/ttyS7 --baud 9600 --talker GN \
     --start "${START_TIME}" --send-offset-ms 120 --print-every 1 \
     --state-file /tmp/rk3588s_joint_sync_state \
     --bind-imx586-trigger --imx586-pps-phase-ns "${PPS_PHASE_NS}"
   ```

5. 启动四路相机。同步程序已经建立 epoch，因此必须关闭相机节点对 trigger enable 的再次控制：

   ```bash
   ros2 launch imx586_camera_ros2 imx586_dual_camera.launch.py \
     state_file:=/tmp/rk3588s_joint_sync_state \
     control_trigger_enable:=false
   ```

## 四路相机与设备映射

默认 launch 配置为：

| 相机 | Video | Subdev |
| --- | --- | --- |
| cam1 | `/dev/video44` | `/dev/v4l-subdev2` |
| cam2 | `/dev/video53` | `/dev/v4l-subdev7` |
| cam3 | `/dev/video62` | `/dev/v4l-subdev12` |
| cam4 | `/dev/video71` | `/dev/v4l-subdev17` |

这只是四颗传感器都成功 probe 时的映射。只要某颗相机 I²C probe 失败，`v4l-subdev*` 编号就可能整体变化，启动前应检查 `media-ctl -p`、`v4l2-ctl --list-devices` 和内核日志。

2026-08-27 的当前启动中，I²C bus 2、4 上两颗 IMX586 返回 `EIO(-5)`，只有 bus 3、6 的两颗成功。这不是其他进程占用；要恢复四路，需要先排查对应相机的供电、复位、排线和 I²C 链路。代码与 launch 本身已经支持四路。

## 录制

相机硬件与 ROS 发布约 10 Hz；外部 H.264 默认约 3 Hz。图像不写入 rosbag：

```bash
SESSION=/userdata/rk3588s/sessions/rk3588s_$(date +%Y%m%d_%H%M%S)
mkdir -p "$SESSION/camera"

ros2 launch imx586_camera_ros2 imx586_dual_camera.launch.py \
  state_file:=/tmp/rk3588s_joint_sync_state \
  control_trigger_enable:=false \
  image_record_dir:="$SESSION/camera" \
  image_record_codec:=h264 \
  image_record_rate_hz:=3.0
```

相机稳定后，在另一终端运行：

```bash
/userdata/rk3588s/tools/record_rk3588s_session.sh \
  --duration 600 --output "$SESSION"
```

MCAP 只包含 `/vanjee_points722`、`/vanjee_lidar_imu_packets`、四路可用相机的 Header 以及可用的 `/tf_static`。不要使用 `ros2 bag record -a`。

## 验证

```bash
python3 /userdata/rk3588s/tools/rk3588s_sync_check.py --duration 60
ros2 bag info "$SESSION/radar_imu_headers"
cat "$SESSION/topics.txt"
cat "$SESSION/manifest.txt"
```

应确认 `missing_trigger=0`、四路相机 Header 接近 10 Hz、bag 中没有 `image_raw`，并检查 WLR-722 点云的有限 `x/y/z` 点数不是 0。

## 许可证

`imx586_camera_ros2` 的包清单声明 Apache-2.0；内核模块声明 GPL；万集配置随其原始 BSD-3-Clause `LICENSE` 一并保存。使用或再发布前请分别遵守对应文件和依赖项目的许可证。
