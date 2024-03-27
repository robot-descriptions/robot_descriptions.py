#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Double pendulum description."""

from os import environ as _environ
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "example-robot-data",
    commit=_environ.pop("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "robots", "double_pendulum_description"
)

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "double_pendulum.urdf")

# Description-specific paths

URDF_PATH_CONTINUOUS: str = _path.join(
    PACKAGE_PATH, "urdf", "double_pendulum_continuous.urdf"
)

URDF_PATH_SIMPLE: str = _path.join(
    PACKAGE_PATH, "urdf", "double_pendulum_simple.urdf"
)
