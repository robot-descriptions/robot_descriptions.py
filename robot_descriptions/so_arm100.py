#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""SO-ARM100 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "SO-ARM100",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "URDF", "SO_5DOF_ARM100_8j_URDF.SLDASM"
)

URDF_PATH: str = _path.join(
    PACKAGE_PATH, "urdf", "SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
)
