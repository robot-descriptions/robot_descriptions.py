#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""PR2 description."""

import warnings as _warnings
from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

_warnings.warn(
    "robot_descriptions.pr2_description is deprecated and will switch to "
    "the official PR2 model in robot_descriptions.py in a later release. "
    "Use robot_descriptions.pr2_official_description now to migrate early.",
    FutureWarning,
    stacklevel=2,
)

REPOSITORY_PATH: str = _clone_to_cache(
    "robot-assets",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "urdfs", "robots", "pr2")

URDF_PATH: str = _path.join(PACKAGE_PATH, "pr2.urdf")
