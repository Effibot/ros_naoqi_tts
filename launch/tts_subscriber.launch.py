import launch.actions
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find Package Directory
    tts_pkg_dir = get_package_share_directory("ros_naoqi_tts")

    # Declare the launch arguments
    ip_declare = DeclareLaunchArgument(
        "nao_ip", default_value="127.0.0.1", description="Ip address of the robot"
    )
    nao_ip = LaunchConfiguration("nao_ip")
    port_declare = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="Port to be used for the connection",
    )
    nao_port = LaunchConfiguration("port", default=9559)
    enc_declare = DeclareLaunchArgument(
        "encoding",
        default_value="utf-8",
        description="Encoding to be used for the connection",
    )
    encoding = LaunchConfiguration("encoding", default="utf-8")

    # Declare the launch nodes
    tts_node = Node(
        package="ros_naoqi_tts",
        executable="tts_node",
        name="tts_node",
        output="screen",
        parameters=[
            {"robot_ip": nao_ip},
            {"robot_port": nao_port},
            {"encoding": encoding},
        ],
    )

    # Create the launch description
    ld = LaunchDescription([ip_declare, port_declare, enc_declare, tts_node])

    return ld
