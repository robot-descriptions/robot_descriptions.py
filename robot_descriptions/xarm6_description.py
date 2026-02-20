#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""xArm6 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "xarm_ros2",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "xarm_description")

XACRO_PATH: str = _path.join(
    PACKAGE_PATH,
    "urdf",
    "xarm_device.urdf.xacro",
)

XACRO_ARGS = {
    "dof": "6",
    "robot_type": "xarm",
}
