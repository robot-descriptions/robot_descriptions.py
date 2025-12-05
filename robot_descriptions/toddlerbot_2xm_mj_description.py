#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""ToddlerBot 2XM MJCF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "mujoco_menagerie",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "toddlerbot_2xm")

MJCF_PATH: str = _path.join(PACKAGE_PATH, "toddlerbot_2xm.xml")

# Description-specific paths

MJCF_PATH_MJX: str = _path.join(PACKAGE_PATH, "toddlerbot_2xm_mjx.xml")

MJCF_PATH_POS: str = _path.join(PACKAGE_PATH, "toddlerbot_2xm_pos.xml")
