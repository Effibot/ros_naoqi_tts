# Desc: Web server for Naoqi TTS

#!/usr/bin/python

import time

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
        # self.executor = None
        # spin the node once
        # self.ros_spinner()
        # get the host and port from the node
        self.host = self.node.host_ip
        self.port = self.node.host_port
        # set the server socket to None
        self.server_socket = None
        # set the client to None
        self.client = None
        self.is_closed = True

    def set_status(self, status: bool):
        self.is_closed = status

    def get_status(self):
        return self.is_closed

    def create_server(self):
        # create the server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # set the socket option to reuse the address
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # bind the socket to the host and port
        self.server_socket.bind((self.host, self.port))
        # set total number of clients
        self.server_socket.listen(1)
        return self.server_socket

    def connection_handler(self):
        try:
            self.node.logger.info(
                f"Waiting for connection on {self.host}:{self.port} [TCP]"
            )
            while True:
                server = self.create_server()
                if self.get_status():
                    client = self.accept_client(server)
                    self.set_status(False)
                    self.do_work(server, client)
                    self.set_status(True)
                else:
                    self.node.logger.info(
                        f"Waiting for connection on {self.host}:{self.port} [TCP]"
                    )
                    time.sleep(1)

        except KeyboardInterrupt:
            self.node.logger.info("Keyboard Interrupt (SIGINT)")
        finally:
            self.close_connection(self.server_socket, self.client)

    def do_work(self, server, client):
        # checks if the client sended all the data
        msg = ""
        while True:
            data = client.recv(4096).decode(encoding=self.node.encoding).strip()
            if data != "JOB_DONE":
                msg += data + "\n"
                self.node.logger.info(f"Received: {data}")
                client.send("ACK\n".encode(self.node.encoding))
            else:
                self.node.logger.info("Received all data")
                client.send("JOB_DONE\n".encode(self.node.encoding))
                self.node.logger.info("Sent: JOB_DONE")
                # forward the message to the tts_node
                self.node.set_msg(msg)
                self.ros_spinner()
                self.close_connection(server, client)
                break

    def accept_client(self, server):
        # accept new connection
        self.client, address = server.accept()
        self.node.logger.info(f"Accepted connection from {address}")
        return self.client

    def close_connection(self, server, client):
        # close the connection if it is open
        if not self.get_status():
            self.set_status(False)
            self.node.logger.info("Closing connection")
            assert isinstance(self.client, socket.socket)
            client.close()
            server.close()
            self.node.logger.info("Connection closed")
        else:
            self.node.logger.info("Connection already closed")

    def ros_spinner(self):
        try:
            self.node.logger.info("Starting ros spinner")
            # rclpy.spin_once(self.node, executor=self.executor)  # spin once
            self.node.forward_message()
        except KeyboardInterrupt:
            self.node.logger.info("Keyboard Interrupt (SIGINT)")
        finally:
            self.node.logger.info("node stopped")

    def set_executor(self, executor):
        self.executor = executor


def run_server(ws: WebServer):
    ws.connection_handler()


global executor


def main(args=None):
    rclpy.init()
    web_server = WebServer()
    # executor = MultiThreadedExecutor()
    # executor.add_node(web_server.node)
    # web_server.set_executor(executor)
    try:
        web_server.node.logger.info("Starting web server")
        web_server.connection_handler()
    except KeyboardInterrupt:
        web_server.node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        web_server.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
