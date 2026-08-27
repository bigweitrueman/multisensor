from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "state_file",
            default_value="/tmp/vanjee_sync_state",
            description="vanjee_pseudo_utc_sync.py key=value state file for epoch mapping.",
        ),
        DeclareLaunchArgument(
            "control_trigger_enable",
            default_value="true",
            description="Whether the camera node should toggle /sys/kernel/imx586_trigger/enable.",
        ),
        DeclareLaunchArgument(
            "image_record_dir",
            default_value="",
            description="Directory for synchronized per-camera H.264 streams and CSV indexes.",
        ),
        DeclareLaunchArgument(
            "image_record_codec",
            default_value="h264",
            description="External image codec: h264 (hardware MPP) or raw (UYVY payload).",
        ),
        DeclareLaunchArgument(
            "image_record_every_n",
            default_value="1",
            description="Write every Nth captured frame to the external image stream.",
        ),
        DeclareLaunchArgument(
            "image_record_rate_hz",
            default_value="3.0",
            description="Target external image write rate; 0 disables rate limiting.",
        ),
        Node(
            package="imx586_camera_ros2",
            executable="imx586_dual_camera_node",
            name="imx586_dual_camera",
            output="screen",
            parameters=[{
                "trigger_dev": "/dev/imx586_trigger",
                "trigger_enable": "/sys/kernel/imx586_trigger/enable",
                "video_dev1": "/dev/video44",
                "video_dev2": "/dev/video53",
                "video_dev3": "/dev/video62",
                "video_dev4": "/dev/video71",
                "subdev1": "/dev/v4l-subdev2",
                "subdev2": "/dev/v4l-subdev7",
                "subdev3": "/dev/v4l-subdev12",
                "subdev4": "/dev/v4l-subdev17",
                "topic1": "/imx586/cam1/image_raw",
                "topic2": "/imx586/cam2/image_raw",
                "topic3": "/imx586/cam3/image_raw",
                "topic4": "/imx586/cam4/image_raw",
                "header_topic1": "/imx586/cam1/header",
                "header_topic2": "/imx586/cam2/header",
                "header_topic3": "/imx586/cam3/header",
                "header_topic4": "/imx586/cam4/header",
                "frame_id1": "imx586_cam1_optical",
                "frame_id2": "imx586_cam2_optical",
                "frame_id3": "imx586_cam3_optical",
                "frame_id4": "imx586_cam4_optical",
                "state_file": LaunchConfiguration("state_file"),
                "encoding": "nv12",
                "image_record_dir": LaunchConfiguration("image_record_dir"),
                "image_record_codec": LaunchConfiguration("image_record_codec"),
                "image_record_every_n": ParameterValue(
                    LaunchConfiguration("image_record_every_n"), value_type=int),
                "image_record_rate_hz": ParameterValue(
                    LaunchConfiguration("image_record_rate_hz"), value_type=float),
                "queue_size": 4,
                "enable_cam3": True,
                "enable_cam4": True,
                "control_trigger_enable": ParameterValue(
                    LaunchConfiguration("control_trigger_enable"), value_type=bool),
                "apply_exposure_center_offset": True,
            }],
        )
    ])
