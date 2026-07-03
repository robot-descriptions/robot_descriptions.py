#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Load a robot description in PyBullet."""

import os
import tempfile
from importlib import import_module  # type: ignore
from typing import Optional

import pybullet

from .._package_dirs import get_package_dirs, resolve_package_uris
from .._xacro import get_urdf_path


def load_robot_description(
    description_name: str,
    commit: Optional[str] = None,
    xacro_args: Optional[dict[str, str]] = None,
    **kwargs,
) -> int:
    """Load a robot description in PyBullet.

    Args:
        description_name: Name of the robot description.
        commit: If specified, check out that commit from the cloned robot
            description repository.
        xacro_args: Optional Xacro arguments overriding the module defaults
            when loading a Xacro-backed description.
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
    if not hasattr(module, "URDF_PATH") and not hasattr(module, "XACRO_PATH"):
        raise ValueError(f"{description_name} is not a URDF/Xacro description")
    urdf_path = get_urdf_path(module, xacro_args=xacro_args)

    pybullet.setAdditionalSearchPath(module.PACKAGE_PATH)

    # PyBullet resolves package:// URIs by walking up from the URDF file
    # location rather than from a set of package directories. This fails for
    # relocatable URDFs cached away from their assets, so we resolve those URIs
    # to absolute paths first. The resolved copy is written next to the source
    # URDF to keep any remaining relative paths valid.
    with open(urdf_path, encoding="utf-8") as urdf_file:
        urdf_string = urdf_file.read()
    resolved_string = resolve_package_uris(
        urdf_string, get_package_dirs(module)
    )
    if resolved_string == urdf_string:
        return pybullet.loadURDF(urdf_path, **kwargs)

    resolved_file = tempfile.NamedTemporaryFile(
        prefix=f"{description_name}-",
        suffix=".urdf",
        dir=os.path.dirname(urdf_path),
        delete=False,
    )
    try:
        resolved_file.write(resolved_string.encode("utf-8"))
        resolved_file.close()
        robot = pybullet.loadURDF(resolved_file.name, **kwargs)
    finally:
        os.unlink(resolved_file.name)
    return robot
