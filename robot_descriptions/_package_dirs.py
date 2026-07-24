#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import os.path
import re
from typing import List

from ._xacro import _map_filename_attrs, get_urdf_path

_PACKAGE_URI_RE = re.compile(r"package://([^/]+)/(.*)")


def get_package_dirs(module) -> List[str]:
    """Get package directories for a given module.

    Args:
        module: Robot description module.

    Returns:
        Package directories.
    """
    package_dirs = [
        module.PACKAGE_PATH,
        module.REPOSITORY_PATH,
        os.path.dirname(module.PACKAGE_PATH),
        os.path.dirname(module.REPOSITORY_PATH),
    ]
    for package_path in getattr(module, "XACRO_PACKAGE_PATHS", {}).values():
        # These packages can live in a separately cloned repository whose
        # directory name differs from the ROS package name
        package_dirs.append(os.path.dirname(package_path))
    if hasattr(module, "URDF_PATH") or hasattr(module, "XACRO_PATH"):
        package_dirs.append(
            os.path.dirname(get_urdf_path(module))  # e.g. laikago_description
        )
    return package_dirs


def resolve_package_uris(urdf_string: str, package_dirs: List[str]) -> str:
    """Rewrite resolvable ``package://`` asset URIs to absolute file paths.

    Args:
        urdf_string: Serialized URDF XML.
        package_dirs: Candidate package directories, as returned by
            :func:`get_package_dirs`.

    Returns:
        URDF XML with resolvable ``package://`` URIs replaced by absolute
        paths; unresolvable references are left unchanged.
    """

    def resolve_one(uri: str) -> str:
        match = _PACKAGE_URI_RE.match(uri)
        if match is None:
            return uri
        package_name, rel_path = match.group(1), match.group(2)
        for package_dir in package_dirs:
            candidate = os.path.join(package_dir, package_name, rel_path)
            if os.path.exists(candidate):
                return candidate
        return uri

    return _map_filename_attrs(urdf_string, resolve_one)
