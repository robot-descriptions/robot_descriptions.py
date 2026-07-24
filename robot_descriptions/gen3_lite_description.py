#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Kinova Gen3 Lite description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "ros2_kortex",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "kortex_description")

XACRO_PATH: str = _path.join(
    PACKAGE_PATH, "robots", "gen3_lite_gen3_lite_2f.xacro"
)

SRDF_PATH: str = _path.join(
    REPOSITORY_PATH,
    "kortex_moveit_config",
    "kinova_gen3_lite_moveit_config",
    "config",
    "gen3_lite.srdf",
)
