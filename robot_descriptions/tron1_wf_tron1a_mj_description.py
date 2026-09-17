#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0

"""TRON1 WF TRON1A MJCF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "tron1_robot_description",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "pointfoot", "WF_TRON1A")

MJCF_PATH: str = _path.join(PACKAGE_PATH, "xml", "robot.xml")
