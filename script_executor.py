# Copyright 2018 NoMagic Sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import socket
from os.path import isfile, join

import jinja2
import rospy

SCRIPT_PORT = 30002


class ScriptExecutor:
    def __init__(self, script_dir, auto_reload=False):
        self.script_dir = script_dir
        self.auto_reload = auto_reload
        self.environment = jinja2.Environment(
            loader=jinja2.FileSystemLoader(self.script_dir),
            auto_reload=auto_reload
        )
        robot_ip = rospy.get_param("/galaxy/hardware/robot_ip", "127.0.0.1")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((robot_ip, SCRIPT_PORT))

    def get_scripts(self):
        return [f for f in os.listdir(self.script_dir) if isfile(join(self.script_dir, f)) and f != '__init__.py']

    def _get_template(self, script_name):
        return self.environment.get_template(script_name)

    def call_script(self, script_name, parameters):
        program = self._get_template(script_name).render(parameters)
        self.socket.sendall(program.encode('latin-1'))
