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
from home_assistant_msgs.msg import *

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
        self.declare_parameter('blacklist', [
            "persistent_notification.",
            "sensor.hacs",
            "person.",
            "binary_sensor.updater",
            "zone.",
        ])
        self.blacklist = self.get_parameter("blacklist")._value
        print(self.blacklist)

        self.ha = HomeAssistant(self.get_parameter("host")._value, self.get_parameter("port")._value, self.get_parameter("token")._value)

        self.timer = self.create_timer(1, self.on_timer)

        self.pubs = {}
        self.ros_types = {}

    def on_timer(self):
        for state in self.ha.states():
            blacklisted = False
            for blacklist_item in self.blacklist:
                if state["entity_id"].startswith(blacklist_item):
                    blacklisted = True
            if blacklisted:
                continue

            if state["entity_id"].startswith("light."):
                ros_type = Light
                ros_value = Light(state = state["state"])
            elif state["entity_id"].startswith("switch."):
                ros_type = Switch
                ros_value = Switch(state = state["state"])
            elif state["entity_id"].startswith("sun."):
                ros_type = Sun
                ros_value = Sun(state = state["state"])
            elif state["entity_id"].startswith("weather."):
                ros_type = WeatherForecast
                ros_value = WeatherForecast(
                    friendly_name = str(state["attributes"].get("friendly_name","")),
                    attribution = str(state["attributes"].get("attribution","")),
                    humidity = float(state["attributes"].get("humidity",0.0)),
                    pressure = float(state["attributes"].get("pressure",0.0)),
                    temperature = float(state["attributes"].get("temperature",0.0)),
                    wind_bearing = float(state["attributes"].get("wind_bearing",0.0)),
                    wind_speed = float(state["attributes"].get("wind_speed",0.0)),
                    forecast = [
                        ForecastData(
                            condition = str(data_point.get("condition", "")),
                            precipitation = float(data_point.get("precipitation", 0.0)),
                            temperature = float(data_point.get("temperature", 0.0)),
                            templow = float(data_point.get("templow", 0.0)),
                            wind_bearing = float(data_point.get("wind_bearing", 0.0)),
                            wind_speed = float(data_point.get("wind_speed", 0.0)),
                        ) for data_point in state["attributes"].get("forecast", [])
                    ],
                    state = state["state"],
                )
            else:
                if state["state"] in ("on", "off"):
                    ros_type = Bool
                    ros_value = Bool(data = {"on": True, "off": False}[state["state"]])
                elif is_int(state["state"]):
                    ros_type = Int64
                    ros_value = Int64(data = int(state["state"]))
                elif is_float(state["state"]):
                    ros_type = Float64
                    ros_value = Float64(data = float(state["state"]))
                else:
                    ros_type = String
                    ros_value = String(data = str(state["state"]))

            topic_name = state["entity_id"].replace(".", "/")

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

            self.pubs[topic_name].publish(ros_value)

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
