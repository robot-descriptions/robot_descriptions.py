# SPDX-License-Identifier: Apache-2.0

"""YAM Ultra v1 MJCF description."""

from os import getenv as _getenv
from os import path as _path

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "i2rt",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(
    REPOSITORY_PATH, "i2rt", "robot_models", "arm", "yam_ultra", "v1"
)

MJCF_PATH: str = _path.join(PACKAGE_PATH, "yam_ultra.xml")
