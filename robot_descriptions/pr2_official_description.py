#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Official PR2 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "pr2_common",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "pr2_description")

XACRO_PATH: str = _path.join(PACKAGE_PATH, "robots", "pr2.urdf.xacro")

XACRO_ARGS = {
    "KINECT1": "false",
    "KINECT2": "false",
}

XACRO_PACKAGE_PATHS: dict[str, str] = {
    "pr2_description": PACKAGE_PATH,
}
