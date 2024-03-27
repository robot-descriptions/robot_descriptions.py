#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Robotiq 2F-85 MJCF description."""

from os import environ as _environ
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "mujoco_menagerie",
    commit=_environ.pop("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "robotiq_2f85")

MJCF_PATH: str = _path.join(PACKAGE_PATH, "2f85.xml")
