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
        "host_ip", default_value="localhost", description="Ip address of host"
    )
    host_ip = LaunchConfiguration("host_ip")
    port_declare = DeclareLaunchArgument(
        "host_port",
        default_value="9090",
        description="Port to be used for the connection",
    )
    host_port = LaunchConfiguration("host_port", default="9090")
    enc_declare = DeclareLaunchArgument(
        "encoding",
        default_value="utf-8",
        description="Encoding to be used for the connection",
    )
    encoding = LaunchConfiguration("encoding", default="utf-8")

    # Declare the launch nodes
    web_server_node = Node(
        package="ros_naoqi_tts",
        executable="web_server",
        name="web_server",
        output="screen",
        parameters=[
            {"host_ip": host_ip},
            {"host_port": host_port},
            {"encoding": encoding},
            {"nao_ip": "127.0.0.1"},
            {"nao_port": "9559"},
        ],
    )

    # Create the launch description
    ld = LaunchDescription([ip_declare, port_declare, enc_declare, web_server_node])

    return ld
