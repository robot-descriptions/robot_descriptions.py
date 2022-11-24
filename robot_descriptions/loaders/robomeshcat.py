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
Load a robot description in `RoboMeshCat
<https://github.com/petrikvladimir/RoboMeshCat>`_.
"""

import os.path
from importlib import import_module  # type: ignore

import robomeshcat


def load_robot_description(description_name: str) -> robomeshcat.Robot:
    """
    Load a robot description in RoboMeshCat.

    Args:
        description_name: Name of the robot description.

    Returns:
        Robot model for RoboMeshCat.
    """
    module = import_module(f"robot_descriptions.{description_name}")

    package_dirs = [
        module.PACKAGE_PATH,
        module.REPOSITORY_PATH,
        os.path.dirname(module.PACKAGE_PATH),
        os.path.dirname(module.REPOSITORY_PATH),
        os.path.dirname(module.URDF_PATH),  # e.g. laikago_description
    ]

    # Workaround to emulate Pinocchio's package_dirs
    # See https://github.com/petrikvladimir/RoboMeshCat/issues/1
    robot = None
    for package_dir in package_dirs:
        if robot is not None:
            break
        try:
            robot = robomeshcat.Robot(
                urdf_path=module.URDF_PATH,
                mesh_folder_path=package_dir,
            )
        except ValueError as e:
            if "could not be found" in str(e):
                continue

    return robot
