#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""SO-ARM101 Parallel Gripper description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "SO-ARM100-101-Parallel-Gripper",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "simulation", "so_arm_101_description"
)

XACRO_PATH: str = _path.join(
    PACKAGE_PATH, "urdf", "so_101.urdf.xacro"
)
