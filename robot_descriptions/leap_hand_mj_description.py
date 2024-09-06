#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""LEAP hand MJCF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "mujoco_menagerie",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "leap_hand")

MJCF_PATH: str = _path.join(PACKAGE_PATH, "right_hand.xml")

# Description-specific paths

MJCF_PATH_LEFT: str = _path.join(PACKAGE_PATH, "left_hand.xml")

MJCF_PATH_RIGHT: str = _path.join(PACKAGE_PATH, "right_hand.xml")
