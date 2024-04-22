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
) -> mujoco.MjModel:
    """Load a robot description in MuJoCo.

    Args:
        description_name: Name of the robot description.
        commit: If specified, check out that commit from the cloned robot
            description repository.

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

    return mujoco.MjModel.from_xml_path(module.MJCF_PATH)
