#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    if commit is not None:
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    robot = pin.RobotWrapper.BuildFromURDF(
        filename=module.URDF_PATH,
        package_dirs=get_package_dirs(module),
        root_joint=root_joint,
    )
    return robot
