#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""UR10 description."""

import warnings as _warnings
from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

_warnings.warn(
    "robot_descriptions.ur10_description is deprecated and will switch to the "
    "official UR10 model in robot_descriptions.py v2. Use "
    "robot_descriptions.ur10_official_description now to migrate early.",
    FutureWarning,
    stacklevel=2,
)

REPOSITORY_PATH: str = _clone_to_cache(
    "example-robot-data",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "robots", "ur_description")

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "ur10_robot.urdf")

# Description-specific paths

URDF_PATH_JOINT_LIMITED: str = _path.join(
    PACKAGE_PATH, "urdf", "ur10_joint_limited_robot.urdf"
)
