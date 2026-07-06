#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""OpenArm v1 URDF description."""

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
    "openarm_v1.0",
    "urdf",
    "openarm_v10.urdf.xacro",
)

XACRO_PACKAGE_PATHS: dict[str, str] = {
    "openarm_description": PACKAGE_PATH,
}

MESH_PATH: str = _path.join(PACKAGE_PATH, "assets")
