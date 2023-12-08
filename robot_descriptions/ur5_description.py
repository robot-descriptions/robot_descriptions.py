#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

"""UR5 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "example-robot-data",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "robots", "ur_description")

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "ur5_robot.urdf")

# Description-specific paths

URDF_PATH_GRIPPER: str = _path.join(PACKAGE_PATH, "urdf", "ur5_gripper.urdf")

URDF_PATH_JOINT_LIMITED: str = _path.join(
    PACKAGE_PATH, "urdf", "ur5_joint_limited_robot.urdf"
)
