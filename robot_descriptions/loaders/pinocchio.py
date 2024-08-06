#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Load a robot description in Pinocchio."""

import os
from importlib import import_module  # type: ignore
from typing import Optional, Union

import pinocchio as pin

from .._package_dirs import get_package_dirs

PinocchioJoint = Union[
    pin.JointModelRX,
    pin.JointModelRY,
    pin.JointModelRZ,
    pin.JointModelPX,
    pin.JointModelPY,
    pin.JointModelPZ,
    pin.JointModelFreeFlyer,
    pin.JointModelSpherical,
    pin.JointModelSphericalZYX,
    pin.JointModelPlanar,
    pin.JointModelTranslation,
]


def load_robot_description(
    description_name: str,
    root_joint: Optional[PinocchioJoint] = None,
    commit: Optional[str] = None,
) -> pin.RobotWrapper:
    """Load a robot description in Pinocchio.

    Args:
        description_name: Name of the robot description.
        root_joint (optional): First joint of the kinematic chain, for example
            a free flyer between the floating base of a mobile robot and an
            inertial frame. Defaults to no joint, i.e., a fixed base.
        commit: If specified, check out that commit from the cloned robot
            description repository.

    Returns:
        Robot model for Pinocchio.
    """
    if commit is not None:  # technical debt, see #31
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    if commit is not None:
        os.environ.pop("ROBOT_DESCRIPTION_COMMIT", None)
    if hasattr(module, "URDF_PATH"):
        robot = pin.RobotWrapper.BuildFromURDF(
            filename=module.URDF_PATH,
            package_dirs=get_package_dirs(module),
            root_joint=root_joint,
        )
    elif hasattr(module, "MJCF_PATH"):
        robot = pin.RobotWrapper.BuildFromMJCF(
            filename=module.MJCF_PATH,
            root_joint=root_joint,
        )
    return robot
