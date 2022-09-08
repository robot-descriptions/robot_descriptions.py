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

"""
Show a robot descriptions specified from the command line, using yourdfpy.

This example requires PyBullet, which is installed by ``pip install pybullet``.
"""

import argparse
import os
from importlib import import_module  # type: ignore

try:
    import pybullet
except ImportError as e:
    raise ImportError(
        "pybullet not found, try ``pip install pybullet``"
    ) from e

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        module = import_module(f"robot_descriptions.{args.name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{args.name}_description")

    client = pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.setAdditionalSearchPath(os.path.dirname(module.URDF_PATH))
    robot = pybullet.loadURDF(os.path.basename(module.URDF_PATH))

    input("Press Enter to close PyBullet and terminate... ")
    pybullet.disconnect()
