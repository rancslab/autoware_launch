# Copyright 2024 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Standalone launch for Ouster lidars using the Nebula OusterRosWrapper.
# Kept separate from nebula_node_container.launch.py because the Ouster wrapper
# uses a nested-parameter API (connection.*, fov.*) and publishes on "points",
# unlike the Velodyne/Robosense/Hesai wrappers.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import yaml


def get_vehicle_info(context):
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        return {x: LaunchConfiguration(x) for x in args}

    nodes = []

    # Base parameters come from the yaml; the launch arguments below override the
    # network settings, model and frame so the same file works across vehicles.
    sensor_param_file = ParameterFile(
        param_file=LaunchConfiguration("sensor_param_path").perform(context),
        allow_substs=True,
    )

    launch_hw = LaunchConfiguration("launch_driver").perform(context).lower() == "true"

    ouster_overrides = {
        "sensor_model": LaunchConfiguration("sensor_model").perform(context),
        "frame_id": LaunchConfiguration("frame_id").perform(context),
        "launch_hw": launch_hw,
        "connection.sensor_ip": LaunchConfiguration("sensor_ip").perform(context),
        "connection.host_ip": LaunchConfiguration("host_ip").perform(context),
        "connection.data_port": int(LaunchConfiguration("data_port").perform(context)),
        "connection.imu_port": int(LaunchConfiguration("imu_port").perform(context)),
        "fov.azimuth.min_deg": float(LaunchConfiguration("cloud_min_angle").perform(context)),
        "fov.azimuth.max_deg": float(LaunchConfiguration("cloud_max_angle").perform(context)),
        "fov.elevation.min_deg": float(LaunchConfiguration("elevation_min_deg").perform(context)),
        "fov.elevation.max_deg": float(LaunchConfiguration("elevation_max_deg").perform(context)),
    }

    nodes.append(
        ComposableNode(
            package="nebula_ouster",
            plugin="nebula::ros::OusterRosWrapper",
            name="ouster_ros_wrapper_node",
            parameters=[sensor_param_file, ouster_overrides],
            remappings=[
                ("points", "pointcloud_raw_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # Pointcloud preprocessor parameters
    distortion_corrector_node_param = ParameterFile(
        param_file=LaunchConfiguration("distortion_correction_node_param_path").perform(context),
        allow_substs=True,
    )
    ring_outlier_filter_node_param = ParameterFile(
        param_file=LaunchConfiguration("ring_outlier_filter_node_param_path").perform(context),
        allow_substs=True,
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True
    cropbox_parameters["processing_time_threshold_sec"] = 0.01

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    mirror_info = get_vehicle_mirror_info(context)
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[distortion_corrector_node_param],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # Ring Outlier Filter is the last component, so control the output frame here.
    if LaunchConfiguration("output_as_sensor_frame").perform(context).lower() == "true":
        ring_outlier_output_frame = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        ring_outlier_output_frame = {"output_frame": ""}  # keep the input frame
    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "pointcloud_before_sync"),
            ],
            parameters=[ring_outlier_filter_node_param, ring_outlier_output_frame],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # Sensor / wrapper
    add_launch_arg("sensor_model", "OS-64", "Ouster sensor model (e.g. OS-64, OS-128)")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("frame_id", "ouster", "frame id")
    add_launch_arg("sensor_ip", "192.168.88.37", "device ip address")
    add_launch_arg("host_ip", "192.168.88.100", "host ip address")
    add_launch_arg("data_port", "7502", "device data port number")
    add_launch_arg("imu_port", "7503", "device imu port number (0 to disable)")
    add_launch_arg("cloud_min_angle", "0", "horizontal FoV start (deg)")
    add_launch_arg("cloud_max_angle", "360", "horizontal FoV end (deg)")
    add_launch_arg("elevation_min_deg", "-45.0", "vertical FoV start (deg)")
    add_launch_arg("elevation_max_deg", "45.0", "vertical FoV end (deg)")
    add_launch_arg(
        "sensor_param_path",
        PathJoinSubstitution(
            [FindPackageShare("common_sensor_launch"), "config", "ouster.param.yaml"]
        ),
        description="path to the Ouster sensor parameter file",
    )

    # Pointcloud preprocessing
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the vehicle mirror position yaml"
    )
    add_launch_arg(
        "distortion_correction_node_param_path",
        PathJoinSubstitution(
            [
                FindPackageShare("common_sensor_launch"),
                "config",
                "distortion_corrector_node.param.yaml",
            ]
        ),
        description="path to parameter file of distortion correction node",
    )
    add_launch_arg(
        "ring_outlier_filter_node_param_path",
        PathJoinSubstitution(
            [
                FindPackageShare("common_sensor_launch"),
                "config",
                "ring_outlier_filter_node.param.yaml",
            ]
        ),
        description="path to parameter file of ring outlier filter node",
    )

    # Container
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("container_name", "ouster_node_container")

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
