import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    video_device = LaunchConfiguration("video_device").perform(context)
    image_width = int(LaunchConfiguration("image_width").perform(context))
    image_height = int(LaunchConfiguration("image_height").perform(context))
    pixel_format = LaunchConfiguration("pixel_format").perform(context)
    output_encoding = LaunchConfiguration("output_encoding").perform(context)
    camera_info_url = LaunchConfiguration("camera_info_url").perform(context)
    camera_frame_id = LaunchConfiguration("camera_frame_id").perform(context)
    camera_container_name = LaunchConfiguration("camera_container_name").perform(context)
    use_intra_process = (
        LaunchConfiguration("use_intra_process").perform(context).lower() == "true"
    )

    camera_params = {
        "video_device": video_device,
        "pixel_format": pixel_format,
        "output_encoding": output_encoding,
        "image_size": [image_width, image_height],
        "camera_frame_id": camera_frame_id,
    }

    if camera_info_url:
        camera_params["camera_info_url"] = camera_info_url

    container = ComposableNodeContainer(
        name=camera_container_name,
        namespace="traffic_light",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="v4l2_camera",
                plugin="v4l2_camera::V4L2Camera",
                name="v4l2_camera",
                parameters=[camera_params],
                extra_arguments=[{"use_intra_process_comms": use_intra_process}],
            ),
        ],
    )

    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(
                name, default_value=default_value, description=description
            )
        )

    add_launch_arg("video_device", "/dev/video0", "V4L2 device path (video_device)")
    add_launch_arg(
        "pixel_format",
        "YUYV",
        "V4L2 FOURCC pixel format: YUYV, UYVY, or GREY",
    )
    add_launch_arg(
        "output_encoding",
        "rgb8",
        "Output image encoding via cv_bridge (e.g. rgb8, bgr8, mono8)",
    )
    add_launch_arg("image_width", "640", "Image width (image_size[0])")
    add_launch_arg("image_height", "480", "Image height (image_size[1])")
    add_launch_arg(
        "camera_frame_id",
        "traffic_light/camera_link",
        "camera_frame_id in CameraInfo/Image header",
    )
    add_launch_arg(
        "camera_info_url",
        "",
        "Optional camera_info yaml, e.g. file:///path/to/camera_info.yaml",
    )
    add_launch_arg(
        "camera_container_name",
        "traffic_light_camera_container",
        "Component container name",
    )
    add_launch_arg("use_intra_process", "true", "Enable intra-process comms")
    add_launch_arg("use_multithread", "false", "Use multithreaded container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )