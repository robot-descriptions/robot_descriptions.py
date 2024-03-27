#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Load a robot description in PyBullet."""

import os
from importlib import import_module  # type: ignore
from typing import Optional

import pybullet


def load_robot_description(
    description_name: str,
    commit: Optional[str] = None,
    **kwargs,
) -> int:
    """Load a robot description in PyBullet.

    Args:
        description_name: Name of the robot description.
        commit: If specified, check out that commit from the cloned robot
            description repository.
        kwargs: arguments passed to pybullet.loadURDF function, including:
            basePosition: 3D position of the base of the robot in world
                coordinates.
            baseOrientation: orientation in quaternion (xyzw) of the base of
                the robot in world coordinates.
            flags: int flags for the URDF loading in pybullet.
            useFixedBase: boolean indicating use a fix joint between world and
                robot base.
            physicsClientId: int indicating the pybullet client id.

    Returns:
        Integer identifier of the robot in PyBullet.
    """
    if commit is not None:  # technical debt, see #31
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    if commit is not None:
        os.environ.pop("ROBOT_DESCRIPTION_COMMIT", None)
    if not hasattr(module, "URDF_PATH"):
        raise ValueError(f"{description_name} is not a URDF description")

    pybullet.setAdditionalSearchPath(module.PACKAGE_PATH)
    robot = pybullet.loadURDF(module.URDF_PATH, **kwargs)
    return robot
