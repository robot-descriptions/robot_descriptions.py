#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""TIAGo description."""

import warnings as _warnings
from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

_warnings.warn(
    "robot_descriptions.tiago_description is deprecated and will "
    "switch to the official TIAGo model in robot_descriptions.py in a "
    "later release. Use robot_descriptions.tiago_official_description "
    "now to migrate early.",
    FutureWarning,
    stacklevel=2,
)

REPOSITORY_PATH: str = _clone_to_cache(
    "example-robot-data",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "robots", "tiago_description")

URDF_PATH: str = _path.join(PACKAGE_PATH, "robots", "tiago.urdf")

# Description-specific paths

URDF_PATH_DUAL: str = _path.join(PACKAGE_PATH, "robots", "tiago_dual.urdf")

URDF_PATH_NO_HAND: str = _path.join(
    PACKAGE_PATH, "robots", "tiago_no_hand.urdf"
)
