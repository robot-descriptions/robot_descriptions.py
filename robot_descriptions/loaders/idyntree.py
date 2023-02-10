#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Giulio Romualdi
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

"""Load a robot description in iDynTree."""

import os
from importlib import import_module  # type: ignore
from typing import List, Optional

import idyntree.swig as idyn

from .._package_dirs import get_package_dirs


def load_robot_description(
    description_name: str,
    joints_list: Optional[List[str]] = None,
    commit: Optional[str] = None,
) -> idyn.Model:
    """Load a robot description in iDynTree.

    Args:
        description_name: Name of the robot description.
        joints_list: Optional parameter containing the list of joints
            considered in the model. If empty, the model will contain all the
            joints specified in the URDF, otherwise a reduced model containing
            only the specified joints is created.
        commit: If specified, check out that commit from the cloned robot
            description repository.

    Returns:
        Identifier of the robot in iDynTree.

    Raises:
        ValueError:
            If the description is not URDF, or iDynTree is unable to load it.
    """
    if commit is not None:
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    if not hasattr(module, "URDF_PATH"):
        raise ValueError(f"{description_name} is not a URDF description")

    model_loader = idyn.ModelLoader()

    if joints_list is None:
        if not model_loader.loadModelFromFile(
            module.URDF_PATH, "urdf", get_package_dirs(module)
        ):
            raise ValueError(
                f"Unable to load {description_name} with iDynTree"
            )
    else:
        if not model_loader.loadReducedModelFromFile(
            module.URDF_PATH, joints_list, "urdf", get_package_dirs(module)
        ):
            raise ValueError(
                f"Unable to load {description_name} with iDynTree"
            )

    robot = model_loader.model().copy()
    return robot
