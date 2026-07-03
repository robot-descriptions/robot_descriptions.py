#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Eve R3 description."""

import hashlib as _hashlib
from os import getenv as _getenv
from os import path as _path
from xml.etree import ElementTree as _ElementTree

from ._cache import clone_to_cache as _clone_to_cache

REPOSITORY_PATH: str = _clone_to_cache(
    "halodi-robot-models",
    commit=_getenv("ROBOT_DESCRIPTION_COMMIT", None),
)

PACKAGE_PATH: str = _path.join(REPOSITORY_PATH, "eve_r3_description")

_ORIGINAL_URDF_PATH: str = _path.join(PACKAGE_PATH, "urdf", "eve_r3.urdf")


def _patch_urdf_limits(urdf_path: str) -> str:
    """Patch invalid negative limits in the upstream URDF."""
    with open(urdf_path, "rb") as urdf_file:
        urdf_hash = _hashlib.sha256(urdf_file.read()).hexdigest()[:16]
    output_path = _path.join(
        _path.dirname(urdf_path), f"eve_r3-{urdf_hash}-v2.urdf"
    )
    if _path.exists(output_path):
        return output_path

    tree = _ElementTree.parse(urdf_path)
    root = tree.getroot()
    for limit in root.findall(".//joint/limit"):
        for attribute in ("effort", "velocity"):
            value = limit.get(attribute)
            if value is not None and float(value) < 0.0:
                # Upstream uses -1 as an unknown/unlimited sentinel, but URDF
                # effort and velocity limits are nonnegative.
                limit.set(attribute, "1e30")

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


URDF_PATH: str = _patch_urdf_limits(_ORIGINAL_URDF_PATH)
