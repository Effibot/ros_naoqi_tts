# Desc: Web server for Naoqi TTS

#!/usr/bin/python

import asyncio
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


class WebServer(Node):
    def __init__(self):
        self.logger = self.get_logger()
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
        websocket.enableTrace(True)
        # self.ws = websocket.WebSocketApp(
        #    f"ws//{self.host_ip}:{self.host_port}",
        #    on_message=self.on_message,
        #    on_error=self.on_error,
        #    on_close=self.on_close,
        # )

    def on_message(self, message):
        msg = String()
        msg.data = str(message, self.encoding)
        self.publisher.publish(msg)  # publish to tts_node
        self.logger.info("received message")  # log message
        # self.ws.send(str(1))  # ACK

    async def handler(self, websocket, path):
        data = await websocket.recv()
        response = f"1"
        self.on_message(data)
        await websocket.send(response)

        def start_server(self):
            server = websockets.serve(self.handler, self.host_ip, self.host_port)
            asyncio.get_event_loop().run_until_complete(server)
            asyncio.get_event_loop().run_forever()

        #

        #
        #    def on_error(self, error):
        #        self.logger.error(error)
        #        self.destroy_node()
        #        sys.exit(1)
        #
        #    def on_close(self):
        #        self.logger.info("closed connection")
        #
        #    def run(self, *args):
        #        time.sleep(10)
        #        self.ws.close()
        #        self.logger.info("thread terminating")
        #
        #    def on_open(self):
        #        self.logger.info("opened connection")

        # threading.Thread(target=self.run, args=()).start()


if __name__ == "__main__":
    rclpy.init()
    web_server = WebServer()
    # web_server.on_open()
    # web_server.ws.run_forever()
    try:
        rclpy.spin(web_server)
    except KeyboardInterrupt:
        web_server.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        web_server.destroy_node()
        rclpy.shutdown()
