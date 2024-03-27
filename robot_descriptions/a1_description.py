#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""A1 description."""

import os

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "unitree_ros",
    commit=(
        os.environ.pop("ROBOT_DESCRIPTION_COMMIT")
        if "ROBOT_DESCRIPTION_COMMIT" in os.environ
        else None
    ),
)

PACKAGE_PATH: str = os.path.join(REPOSITORY_PATH, "robots", "a1_description")

URDF_PATH: str = os.path.join(PACKAGE_PATH, "urdf", "a1.urdf")
