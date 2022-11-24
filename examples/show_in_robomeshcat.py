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
Load a robot description, specified from the command line, in RoboMeshCat.

This example uses RoboMeshCat: https://github.com/petrikvladimir/RoboMeshCat
"""

import argparse

import numpy as np
import robomeshcat

from robot_descriptions.loaders.robomeshcat import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    scene = robomeshcat.Scene()
    scene.add_robot(robot)

    with scene.animation(fps=1):
        scene.render()
        robot[0] = -0.5 * np.pi
        scene.render()

        for t in np.linspace(0, 2 * np.pi, 10):
            scene.camera_pos = np.array([np.sin(t), np.cos(t), 1.0])
            scene.render()

        scene.reset_camera()  # back to user interaction
        scene.render()
