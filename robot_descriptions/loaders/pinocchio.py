#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
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

"""
Load a robot description in Pinocchio.
"""

import os.path
from importlib import import_module  # type: ignore
from typing import Union

import pinocchio as pin

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
    root_joint_type: PinocchioJoint = pin.JointModelFreeFlyer,
) -> pin.RobotWrapper:
    """
    Load a robot description in Pinocchio.

    Args:
        description_name: Name of the robot description.
        root_joint_type: Type of the first joint of the kinematic chain,
            typically a free flyer (a.k.a. floating base) for mobile robots.

    Returns:
        Robot models for Pinocchio.
    """
    module = import_module(f"robot_descriptions.{description_name}")

    package_dirs = [
        module.PACKAGE_PATH,
        module.REPOSITORY_PATH,
        os.path.dirname(module.PACKAGE_PATH),
        os.path.dirname(module.REPOSITORY_PATH),
        os.path.dirname(module.URDF_PATH),  # e.g. laikago_description
    ]

    robot = pin.RobotWrapper.BuildFromURDF(
        filename=module.URDF_PATH,
        package_dirs=package_dirs,
        root_joint=root_joint_type(),
    )

    return robot
