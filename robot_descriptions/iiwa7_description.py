#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""iiwa 7 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "differentiable-robot-model",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "diff_robot_data", "kuka_iiwa")

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "iiwa7.urdf")

# Description-specific paths

URDF_PATH_ALLEGRO: str = _path.join(PACKAGE_PATH, "urdf", "iiwa7_allegro.urdf")
