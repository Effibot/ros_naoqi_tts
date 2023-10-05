import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String


class TTSNode(Node):
    def __init__(self):
        super().__init__(
            "tts_node",
        )  # type: ignore

        # get logger
        self.logger = rclpy.logging.get_logger("tts_node")  # type: ignore
        # declare params
        self.declare_parameter(
            "robot_ip",
            "127.0.0.1",
            ParameterDescriptor(
                description="Ip address of the robot, default: 127.0.0.1"
            ),
        )
        self.declare_parameter(
            "robot_port",
            9559,
            ParameterDescriptor(description="Port of the robot, default: 9559"),
        )
        self.declare_parameter(
            "encoding",
            "utf-8",
            ParameterDescriptor(description="Encoding to use, default: utf-8"),
        )

        self.ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.port = self.get_parameter("robot_port").get_parameter_value().integer_value
        self.encoding = (
            self.get_parameter("encoding").get_parameter_value().string_value
        )

        self.tts_core_script = os.path.join(
            get_package_share_directory("ros_naoqi_tts"),
            "script",
            "tts_core.py",
        )

        self.logger.info(f"tts_core_script path is: {self.tts_core_script}")

        self.subscription = self.create_subscription(
            String, "web_server", self.tts_callback, 10
        )

    def get_script(self):
        return self.tts_core_script

    def run_tts_service(self, ip: str, port: int, text: str, encoding: str):
        subprocess.run(
            [
                "python2",
                self.get_script(),
                f"--ip={ip}",
                f"--port={port}",
                f"--text={text}",
                f"--encoding={encoding}",
            ]
        )

    def tts_callback(self, msg: String):
        # received the text to say -> run the service
        self.logger.info("Received: message, starting ALTextToSpeech service")
        self.run_tts_service(self.ip, self.port, msg.data, self.encoding)


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        tts_node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
