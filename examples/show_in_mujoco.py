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
Show a robot description, specified from the command line, using MuJoCo.

This example requires MuJoCo, which is installed by ``pip install mujoco``, and
the MuJoCo viewer installed by ``pip install mujoco-python-viewer``.
"""

import argparse

import mujoco

try:
    import mujoco_viewer
except ImportError as e:
    raise ImportError(
        "MuJoCo viewer not found, " "try ``pip install mujoco-python-viewer``"
    ) from e

from robot_descriptions.loaders.mujoco import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        model = load_robot_description(args.name)
    except ModuleNotFoundError:
        model = load_robot_description(f"{args.name}_mj_description")

    data = mujoco.MjData(model)
    viewer = mujoco_viewer.MujocoViewer(model, data)
    mujoco.mj_step(model, data)  # step at least once to load model in viewer
    while viewer.is_alive:
        viewer.render()
    viewer.close()
