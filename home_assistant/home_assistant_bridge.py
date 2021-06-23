#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
import re
import os
import sys
import time
import requests

from std_msgs.msg import String, Int32, Int64, Float32, Float64, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Temperature

try:
    # when executed as module
    from .home_assistant import HomeAssistant
except:
    # when excecuted directly
    from home_assistant import HomeAssistant

with open(os.path.expanduser("~/private/credentials/home_assistant.json")) as f:
    config = json.loads(f.read())

class HomeAssistantNode(Node):
    def __init__(self, node_name = "home_assistant_node"):
        super().__init__(node_name = node_name)
        self.log = self.get_logger()
        self.clock = self.get_clock()

        self.declare_parameter('host', "192.168.1.100")
        self.declare_parameter('port', 8123)
        self.declare_parameter('token', config["token"])
        self.ha = HomeAssistant(self.get_parameter("host")._value, self.get_parameter("port")._value, self.get_parameter("token")._value)

        self.timer = self.create_timer(1, self.on_timer)

        self.pubs = {}
        self.ros_types = {}

    def on_timer(self):
        for state in self.ha.states():
            print(state)
            topic_name = state["entity_id"].replace(".", "/")

            if state["state"] in ("on", "off"):
                ros_type = Bool
                ros_value = {"on": True, "off": False}[state["state"]]
            elif is_int(state["state"]):
                ros_type = Int64
                ros_value = int(state["state"])
            elif is_float(state["state"]):
                ros_type = Float64
                ros_value = float(state["state"])
            else:
                ros_type = String
                ros_value = str(state["state"])

            if topic_name not in self.pubs:
                try:
                    self.pubs[topic_name] = self.create_publisher(ros_type, topic_name, 10)
                    self.ros_types[topic_name] = ros_type
                except rclpy.exceptions.InvalidTopicNameException:
                    self.log.warn("Cannot create topic name %s" % topic_name)
                    continue

            if ros_type != self.ros_types[topic_name]:
                self.log.warn("type changed for %s (%s -> %s)" % (topic_name, str(self.ros_types[topic_name], str(ros_type))))
                continue

            msg = self.ros_types[topic_name]()
            print(topic_name, self.ros_types[topic_name], ros_value)
            msg.data = ros_value
            self.pubs[topic_name].publish(msg)

def is_int(s):
    try:
        int(s)
        return True
    except:
        pass
    return False

def is_float(s):
    try:
        float(s)
        return True
    except:
        pass
    return False

def main(args=None):
    rclpy.init(args=args)
    node = HomeAssistantNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
