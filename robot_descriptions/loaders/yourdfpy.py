#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Load a robot description in yourdfpy."""

import os
from importlib import import_module  # type: ignore
from typing import Optional

try:
    import yourdfpy
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "This feature requires 'yourdfpy', "
        "which can be installed by `pip install yourdfpy`"
    ) from e


def load_robot_description(
    description_name: str,
    commit: Optional[str] = None,
    **kwargs,
) -> yourdfpy.URDF:
    """Load a robot description in yourdfpy.

    Args:
        description_name: Name of the robot description.
        commit: If specified, check out that commit from the cloned robot
            description repository.
        kwargs: arguments passed to yourdfpy.URDF.load function, including:
            build_scene_graph: Whether to build a scene graph from visual
                elements.
            build_collision_scene_graph: Whether to build a scene graph from
                collision elements.
            load_meshes: Whether to load the meshes for the visual elements.
            load_collision_meshes: Whether to load the meshes for the collision
                elements.

    Returns:
        Robot model for yourdfpy.
    """
    if commit is not None:  # technical debt, see #31
        os.environ["ROBOT_DESCRIPTION_COMMIT"] = commit
    module = import_module(f"robot_descriptions.{description_name}")
    if commit is not None:
        os.environ.pop("ROBOT_DESCRIPTION_COMMIT", None)
    if not hasattr(module, "URDF_PATH"):
        raise ValueError(f"{description_name} is not a URDF description")

    return yourdfpy.URDF.load(
        module.URDF_PATH,
        mesh_dir=module.PACKAGE_PATH,
        **kwargs,
    )
