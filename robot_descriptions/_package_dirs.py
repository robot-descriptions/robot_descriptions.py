#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Giulio Romualdi

import os.path
from typing import List

from ._xacro import get_urdf_path


def get_package_dirs(module) -> List[str]:
    """Get package directories for a given module.

    Args:
        module: Robot description module.

    Returns:
        Package directories.
    """
    package_dirs = [
        module.PACKAGE_PATH,
        module.REPOSITORY_PATH,
        os.path.dirname(module.PACKAGE_PATH),
        os.path.dirname(module.REPOSITORY_PATH),
    ]
    if hasattr(module, "URDF_PATH") or hasattr(module, "XACRO_PATH"):
        package_dirs.append(
            os.path.dirname(get_urdf_path(module))  # e.g. laikago_description
        )
    return package_dirs
