#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Aero Hand Open description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "aero-hand-open",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH,
    "ros2",
    "src",
    "aero_hand_open_description",
)

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "aero_hand_open_left.urdf")

# Description-specific paths

URDF_PATH_LEFT: str = _path.join(PACKAGE_PATH, "aero_hand_open_left.urdf")

URDF_PATH_RIGHT: str = _path.join(PACKAGE_PATH, "aero_hand_open_left.urdf")
