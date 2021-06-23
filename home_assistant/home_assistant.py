#!/usr/bin/env python3

import json
import socket
import psutil
import re
import os
import sys
import time
import subprocess
import requests

class HomeAssistant(object):
    def __init__(self, host, port, token, protocol = "http"):
        self.protocol = protocol
        self.host = host
        self.port = port
        self.token = token

    def states(self):
        return self.api_get("states")

    def api_get(self, path):
        url = "%s://%s:%d/api/%s" % (self.protocol, self.host, self.port, path)
        headers = {
            "Authorization": "Bearer %s" % self.token,
            "content-type": "application/json",
        }
        response = requests.get(url, headers=headers)
        return json.loads(response.text)

    def api_post(self, path, body):
        url = "%s://%s:%d/api/%s" % (self.protocol, self.host, self.port, path)
        headers = {
            "Authorization": "Bearer %s" % self.token,
            "content-type": "application/json",
        }
        response = requests.post(url, headers=headers, json=body)
        return json.loads(response.text)
