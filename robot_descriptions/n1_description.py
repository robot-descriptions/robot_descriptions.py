#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Fourier N1 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "Wiki-GRx-Models",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "N1")

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "N1_raw.urdf")
