#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Toyota HSR-C description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "hsrb_common",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "hsrc_description")

XACRO_PATH: str = _path.join(PACKAGE_PATH, "robots", "hsrc1s.urdf.xacro")

XACRO_PACKAGE_PATHS: dict[str, str] = {
    "hsrc_description": PACKAGE_PATH,
    "hsrb_description": _path.join(REPOSITORY_PATH, "hsrb_description"),
    "hsrb_parts_description": _path.join(
        REPOSITORY_PATH, "hsrb_parts_description"
    ),
}
