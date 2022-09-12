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
Load a robot description in PyBullet.
"""

from importlib import import_module  # type: ignore

import pybullet


def load_robot_description(description_name: str) -> int:
    """
    Load a robot description in Pinocchio.

    Args:
        description_name: Name of the robot description.

    Returns:
        Identifier of the robot in PyBullet.
    """
    module = import_module(f"robot_descriptions.{description_name}")
    if not hasattr(module, "URDF_PATH"):
        raise ValueError(f"{description_name} is not a URDF description")

    pybullet.setAdditionalSearchPath(module.PACKAGE_PATH)
    robot = pybullet.loadURDF(module.URDF_PATH)
    return robot
