# Desc: Web server for Naoqi TTS

#!/usr/bin/python

import asyncio
import datetime
import sys
import threading
import time
from concurrent.futures import thread

import rclpy
import websocket
import websockets
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String
from websocket import create_connection


class WebNode(Node):
    def __init__(self):
        super().__init__(
            "web_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )  # type: ignore
        self.logger = rclpy.logging.get_logger("WS_node")  # type: ignore
        # declare params
        self.declare_parameter(
            "host_ip",
            "localhost",
            ParameterDescriptor(
                description="Ip address of the machine that executes the server, default:localhost"
            ),
        )
        self.declare_parameter(
            "host_port",
            "6000",
            ParameterDescriptor(description="Port of the server, default: 6000"),
        )
        self.declare_parameter(
            "encoding",
            "utf-8",
            ParameterDescriptor(description="Encoding to use, default: utf-8"),
        )
        self.declare_parameter(
            "nao_ip",
            "127.0.0.1",
            ParameterDescriptor(
                description="Ip address of Android Tablet on Robot, default: 127.0.0.1"
            ),
        )
        self.declare_parameter(
            "nao_port",
            "9959",
            ParameterDescriptor(description="Port of the robot, default: 9959"),
        )
        self.host_ip = self.get_parameter("host_ip").get_parameter_value().string_value
        self.host_port = (
            self.get_parameter("host_port").get_parameter_value().integer_value
        )
        self.encoding = (
            self.get_parameter("encoding").get_parameter_value().string_value
        )
        self.nao_ip = self.get_parameter("nao_ip").get_parameter_value().string_value
        self.nao_port = (
            self.get_parameter("nao_port").get_parameter_value().integer_value
        )
        # declare publisher for tts_node
        self.publisher = self.create_publisher(String, "web_server", 10)


class WebServer:
    def __init__(self) -> None:
        # get reference to node class
        self.node = WebNode()
        websocket.enableTrace(True)
        self.ws = websocket.WebSocketApp(
            f"ws://{self.node.host_ip}:{self.node.host_port}",
            on_message=lambda ws, msg: self.on_message(ws, msg),
            on_error=lambda ws, error: self.on_error(ws, error),
            on_close=lambda ws: self.on_close(ws),
        )
        self.client_count = 0
        self.start_time = datetime.datetime.now()
        self.current_time = datetime.datetime.now()
        self.on_open_function = lambda ws: self.on_open(ws)
        self.is_open = False

    def on_message(self, ws, message):
        self.node.logger.info("received message")  # log message
        msg = String()
        msg.data = str(message, self.node.encoding)
        self.node.publisher.publish(msg)  # publish to tts_node
        self.node.logger.info("published message")  # log message

    def on_error(self, ws, error):
        self.node.logger.error(error)
        self.node.destroy_node()
        sys.exit(1)

    def on_close(self, ws):
        self.node.logger.info(
            f"closed connection at {self.node.host_ip}:{self.node.host_port}"
        )
        self.client_count -= 1

    def run(self, *args):
        time.sleep(1)
        self.ws.send(f"1")
        while self.client_count > 0:
            time.sleep(10)
            self.current_time = datetime.datetime.now()
            uptime = self.current_time - self.start_time
            self.node.logger.info(f"The server has been up for: {uptime}")
        self.node.logger.info("no active clients closing connection")
        self.ws.close()

    def on_open(self, ws):
        self.node.logger.info(
            f"opened connection at {self.node.host_ip}:{self.node.host_port}"
        )
        self.client_count += 1
        threading.Thread(target=self.run, args=()).start()

    def ros_spinner(self):
        try:
            rclpy.spin_once(self.node)  # spin once
        except KeyboardInterrupt:
            self.node.logger.info("Keyboard Interrupt (SIGINT)")
        finally:
            self.node.logger.info("node stopped")


def main(args=None):
    rclpy.init()
    web_server = WebServer()
    web_server.ws.on_open = web_server.on_open_function
    try:
        web_server.is_open = web_server.ws.run_forever()
        # web_server.start_server()
    except KeyboardInterrupt:
        web_server.node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        web_server.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
