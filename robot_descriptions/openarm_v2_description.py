#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""OpenArm v2 URDF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "openarm_description",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = REPOSITORY_PATH

XACRO_PATH: str = _path.join(
    PACKAGE_PATH,
    "assets",
    "robot",
    "openarm_v2.0",
    "urdf",
    "openarm_v20.urdf.xacro",
)

XACRO_PACKAGE_PATHS: dict[str, str] = {
    "openarm_description": PACKAGE_PATH,
}

XACRO_ARGS_LEFT_ARM = {
    "robot_preset": "left_arm",
}

XACRO_ARGS_LEFT_ARM_WITH_PINCH_GRIPPER = {
    "robot_preset": "left_arm_with_pinch_gripper",
}

XACRO_ARGS_RIGHT_ARM = {
    "robot_preset": "right_arm",
}

XACRO_ARGS_RIGHT_ARM_WITH_PINCH_GRIPPER = {
    "robot_preset": "right_arm_with_pinch_gripper",
}

MESH_PATH: str = _path.join(PACKAGE_PATH, "assets")
