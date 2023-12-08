#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

"""TriFingerEdu description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "differentiable-robot-model",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "diff_robot_data", "trifinger_edu_description"
)

URDF_PATH: str = _path.join(PACKAGE_PATH, "trifinger_edu.urdf")
