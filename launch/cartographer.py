import sys

from launch import LaunchDescription
from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ## ***** File paths ******
    # pkg_share = FindPackageShare("cartographer_ros").find("cartographer_ros")
    # urdf_dir = os.path.join(pkg_share, "urdf")
    # urdf_file = os.path.join(urdf_dir, "backpack_3d.urdf")
    # with open(urdf_file, "r") as infp:
    #     robot_desc = infp.read()

    # robot_state_publisher_node = Node(
    #     package = "robot_state_publisher",
    #     executable = "robot_state_publisher",
    #     output = "screen",
    #     parameters = [
    #         {"robot_description": robot_desc},
    #     ],
    # )

    base_tf2_laser = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["--frame-id", "base_link", "--child-frame-id", "laser_link"],
    )

    laser_tf2_imu = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["--y", "0.01", "--z", "-0.01", "--qy", "1.00", "--qw", "0.00",
                     "--frame-id", "laser_link", "--child-frame-id", "imu_link"],
    )

    hls_lfcd_lds_publisher_node = Node(
        package = "hls_lfcd_lds_driver",
        executable = "hlds_laser_publisher",
        name = "hlds_laser_publisher",
        parameters = [
            {"port": "/dev/ttyUSB0",
             "frame_id": "laser_link"}],
        output = "screen",
    )

    imu_node = Node(
        executable = "meson-build-debug/nodes/icm20600_publisher",
    )

    controller_node = Node(
        executable = sys.executable,
        arguments=['nodes/controller.py'],
    )

    cartographer_node = Node(
        package = "cartographer_ros",
        executable = "cartographer_node",
        output = "screen",
        arguments = [
            "-configuration_directory", "params/",
            "-configuration_basename", "lds_2d.lua"],
        remappings = [
            ("/imu", "/imu"),
            ("/scan", "/scan"),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package = "cartographer_ros",
        executable = "cartographer_occupancy_grid_node",
        parameters = [
            {"resolution": 0.05},
        ],
    )

    return LaunchDescription([
        # robot_state_publisher_node,
        base_tf2_laser,
        laser_tf2_imu,
        hls_lfcd_lds_publisher_node,
        imu_node,
        controller_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
