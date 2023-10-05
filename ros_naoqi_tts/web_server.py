# Desc: Web server for Naoqi TTS

#!/usr/bin/python

import asyncio
import datetime
import sys
import threading
import time
from concurrent.futures import thread

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String


class WebNode(Node):
    def __init__(self):
        super().__init__("web_node")  # type: ignore
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
            "9559",
            ParameterDescriptor(description="Port of the robot, default: 9559"),
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

        # declare timer callback to force the node to spin when no messages are received by the server
        self.timer_period = 0.5
        self.msg = String()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # declare boolean var to check if the node spin at least once
        self.is_spin_once = False

    def set_msg(self, msg):
        if isinstance(msg, str):
            self.msg = String(data=msg)
        else:
            self.msg = msg

    def get_msg(self):
        return self.msg

    def timer_callback(self):
        # check if the node has spin at least once
        if not self.is_spin_once:
            self.logger.info("Spinning once")
            self.is_spin_once = True
        else:
            # just forward the message
            self.forward_message()

    def forward_message(self):
        self.logger.info("received message")  # log message
        self.publisher.publish(self.msg)  # publish to tts_node
        self.logger.info("published message")  # log message


import socket


class WebServer:
    def __init__(self) -> None:
        # get reference to node class
        self.node = WebNode()
        # spin the node once
        self.ros_spinner()
        # create the server
        self.host = self.node.host_ip
        self.port = self.node.host_port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        # set total number of clients
        self.server_socket.listen(1)
        self.client_count = 0
        self.client = None
        self.start_time = datetime.datetime.now()
        self.current_time = None
        self.is_closed = True
        self.msg = ""

    def set_status(self, status: bool):
        self.is_closed = status

    def get_status(self):
        return self.is_closed

    def accept_client(self):
        try:
            # accept new connection
            self.node.logger.info(
                f"Waiting for connection on {self.host}:{self.port} [TCP]"
            )
            self.client, address = self.server_socket.accept()
            self.node.logger.info(f"Accepted connection from {address}")

            # checks if the client sended all the data
            while True:
                data = self.client.recv(1024).decode(self.node.encoding)
                if data != "JOB_DONE":
                    self.msg += data + "\n"
                    self.node.logger.info(f"Received: {data}")
                    self.client.send("ACK".encode(self.node.encoding))
                else:
                    self.node.logger.info("Received all data")
                    self.client.send("JOB_DONE".encode(self.node.encoding))
                    self.node.logger.info("Sent: JOB_DONE")
                    # forward the message to the tts_node
                    self.node.set_msg(self.msg)
                    self.ros_spinner()
                    break
        except KeyboardInterrupt:
            self.node.logger.info("Keyboard Interrupt (SIGINT)")
        finally:
            self.close_connection()

    def close_connection(self):
        # close the connection if it is open
        if self.get_status():
            self.set_status(False)
            self.node.logger.info("Closing connection")
            assert isinstance(self.client, socket.socket)
            self.client.close()
            self.server_socket.close()
            self.node.logger.info("Connection closed")
        else:
            self.node.logger.info("Connection already closed")

    def ros_spinner(self):
        try:
            self.node.logger.info("Starting ros spinner")
            rclpy.spin_once(self.node)  # spin once
        except KeyboardInterrupt:
            self.node.logger.info("Keyboard Interrupt (SIGINT)")
        finally:
            self.node.logger.info("node stopped")


def main(args=None):
    rclpy.init()
    web_server = WebServer()
    try:
        web_server.node.logger.info("Starting web server")
        web_server.accept_client()
    except KeyboardInterrupt:
        web_server.node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        web_server.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
