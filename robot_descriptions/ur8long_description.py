#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""UR8 Long description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "Universal_Robots_ROS2_Description",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = REPOSITORY_PATH

XACRO_PATH: str = _path.join(PACKAGE_PATH, "urdf", "ur.urdf.xacro")

XACRO_ARGS = {
    "ur_type": "ur8long",
    "name": "ur8long",
}
