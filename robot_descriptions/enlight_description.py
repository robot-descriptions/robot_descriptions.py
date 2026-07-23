#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Flexiv Enlight description.

The default variant is the single-arm Enlight-L. Upstream also supports
the dual-arm Enlight-LL variant through ``xacro_args={"robot_type":
"Enlight-LL"}``.
"""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "flexiv_description_v2",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = REPOSITORY_PATH

XACRO_PATH: str = _path.join(PACKAGE_PATH, "urdf", "flexiv.urdf.xacro")

XACRO_PACKAGE_PATHS = {
    "flexiv_description": PACKAGE_PATH,
}

XACRO_ARGS = {
    "robot_type": "Enlight-L",
}
