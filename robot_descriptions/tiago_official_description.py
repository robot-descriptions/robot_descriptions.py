#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Official TIAGo description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "tiago_robot",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PAL_HEY5_REPOSITORY_PATH: str = _clone_to_cache(
    "pal_hey5",
)

PAL_URDF_UTILS_REPOSITORY_PATH: str = _clone_to_cache(
    "pal_urdf_utils",
)

PMB2_REPOSITORY_PATH: str = _clone_to_cache(
    "pmb2_robot",
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "tiago_description")

PAL_HEY5_PACKAGE_PATH: str = _path.join(
    PAL_HEY5_REPOSITORY_PATH, "pal_hey5_description"
)

PMB2_PACKAGE_PATH: str = _path.join(PMB2_REPOSITORY_PATH, "pmb2_description")

XACRO_PATH: str = _path.join(PACKAGE_PATH, "robots", "tiago.urdf.xacro")

XACRO_PACKAGE_PATHS: dict[str, str] = {
    "pal_hey5_description": PAL_HEY5_PACKAGE_PATH,
    "pal_urdf_utils": PAL_URDF_UTILS_REPOSITORY_PATH,
    "pmb2_description": PMB2_PACKAGE_PATH,
    "tiago_description": PACKAGE_PATH,  # Needed
    # because the module name is different from the package name
}
