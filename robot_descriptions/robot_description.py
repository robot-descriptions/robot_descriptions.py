#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
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


class RobotDescription:

    """
    Robot description.

    Attributes:
        path: Path to the description folder in the cache.
        meshes_path: Path to the meshes folder.
        urdf_path: Path to the robot's URDF.
    """

    def __init__(self):
        path = os.path.dirname(os.path.realpath(__file__))
        self.path = path
        self.meshes_path = os.path.join(path, "meshes")
        self.urdf_path = os.path.join(path, "urdf", "upkie.urdf")
