#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""GR-1 description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "Wiki-GRx-Models",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "GRX", "GR1", "GR1T1")

URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "GR1T1_nohand.urdf")

URDF_PATH_NO_HAND: str = _path.join(PACKAGE_PATH, "urdf", "GR1T1_no_hand.urdf")

URDF_PATH_FOURIER_HAND: str = _path.join(
    PACKAGE_PATH, "urdf", "GR1T1_fourier_hand_6dof.urdf"
)

URDF_PATH_INSPIRE_HAND: str = _path.join(
    PACKAGE_PATH, "urdf", "GR1T1_inspire_hand.urdf"
)

URDF_PATH_JAW: str = _path.join(PACKAGE_PATH, "GR1T1_jaw.urdf")
