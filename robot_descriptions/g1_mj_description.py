#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""G1 MJCF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "mujoco_menagerie",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "unitree_g1")

MJCF_PATH: str = _path.join(PACKAGE_PATH, "g1.xml")

MJCF_PATH_WITH_HANDS: str = _path.join(PACKAGE_PATH, "g1_with_hands.xml")
