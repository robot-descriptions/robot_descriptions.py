#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Kinova Jaco Gen3 description."""

from os import environ as _environ
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "kinova_mj_description",
    commit=_environ.pop("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH)

MJCF_PATH: str = _path.join(PACKAGE_PATH, "xml", "gen3_7dof_mujoco.xml")
