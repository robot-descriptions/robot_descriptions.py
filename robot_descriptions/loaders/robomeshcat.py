#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Load a robot description in `RoboMeshCat`_.

.. _RoboMeshCat: https://github.com/petrikvladimir/RoboMeshCat
"""

import os
from importlib import import_module  # type: ignore
from typing import Optional

import robomeshcat

from .pinocchio import get_package_dirs


def load_robot_description(
    description_name: str,
    commit: Optional[str] = None,
) -> robomeshcat.Robot:
    """Load a robot description in RoboMeshCat.

    Args:
        description_name: Name of the robot description.
        commit: If specified, check out that commit from the cloned robot
            description repository.

    Returns:
        Robot model for RoboMeshCat.
    """
    if commit is not None:  # technical debt, see #31
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    if commit is not None:
        os.environ.pop("ROBOT_DESCRIPTION_COMMIT", None)
    robot = robomeshcat.Robot(
        urdf_path=module.URDF_PATH,
        mesh_folder_path=get_package_dirs(module),
    )
    return robot
