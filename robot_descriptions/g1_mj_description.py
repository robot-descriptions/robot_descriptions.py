#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unitree G1 MJCF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "unitree_ros",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "robots", "g1_description")

MJCF_PATH: str = _path.join(PACKAGE_PATH, "g1.xml")
