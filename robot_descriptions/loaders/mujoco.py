#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Load a robot description in MuJoCo."""

import os
from importlib import import_module  # type: ignore
from typing import Optional

import mujoco


def load_robot_description(
    description_name: str,
    commit: Optional[str] = None,
    variant: Optional[str] = None,
) -> mujoco.MjModel:
    """Load a robot description in MuJoCo.

    Args:
        description_name: Name of the robot description.
        commit: If specified, check out that commit from the cloned robot
            description repository.
        variant: If specified, load a specific variant of the robot model.

    Returns:
        Robot model for MuJoCo.
    """
    if commit is not None:  # technical debt, see #31
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    if commit is not None:
        os.environ.pop("ROBOT_DESCRIPTION_COMMIT", None)
    if not hasattr(module, "MJCF_PATH"):
        raise ValueError(f"{description_name} is not an MJCF description")

    def load_model_from_path(path):
        try:
            return mujoco.MjModel.from_xml_path(path)
        except ValueError:
            print(f"{path} not found. Loading default robot model.")
            return None

    model_path = module.MJCF_PATH
    if variant is not None:
        path_parts = model_path.split(os.sep)
        path_parts[-1] = f"{variant}.xml"
        variant_path = os.sep.join(path_parts)
        model = load_model_from_path(variant_path)
        if model:
            return model
    return load_model_from_path(model_path)
