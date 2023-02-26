#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tk_chatroom.srv import Message


class Server(Node):

    def __init__(self):
        super().__init__(node_name="server")
        self.get_logger().info(message="server start")
        self.pub_chatroom = self.create_publisher(msg_type=String, topic="/chatroom", qos_profile=10)
        self.create_service(srv_type=Message, srv_name=f"{self.get_name()}/send_message", callback=self._service_callback)

    def _service_callback(self, request, response):
        msg = String()
        msg.data = f"{request.name}: {request.message}"
        self.pub_chatroom.publish(msg=msg)
        return response


def main():

    try:
        rclpy.init(args=None)
        _checker = rclpy.create_node(node_name="_checker")
        if "server" in _checker.get_node_names():
            print("Node server exists")
            return
        server = Server()
        rclpy.spin(node=server)
    except KeyboardInterrupt:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
