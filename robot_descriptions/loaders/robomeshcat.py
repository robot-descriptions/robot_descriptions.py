#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
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

"""Load a robot description in `RoboMeshCat`_.

.. _RoboMeshCat: https://github.com/petrikvladimir/RoboMeshCat
"""

import os
from importlib import import_module  # type: ignore
from typing import Optional

import robomeshcat

from .pinocchio import get_package_dirs


def load_robot_description(
    description_name: str,
    commit: Optional[str] = None,
) -> robomeshcat.Robot:
    """Load a robot description in RoboMeshCat.

    Args:
        description_name: Name of the robot description.
        commit: If specified, check out that commit from the cloned robot
            description repository.

    Returns:
        Robot model for RoboMeshCat.
    """
    if commit is not None:
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    robot = robomeshcat.Robot(
        urdf_path=module.URDF_PATH,
        mesh_folder_path=get_package_dirs(module),
    )
    return robot
