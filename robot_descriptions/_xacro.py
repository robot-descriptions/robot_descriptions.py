#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Inria

"""Utilities to resolve URDF files from Xacro sources."""

import hashlib
import json
import os
import tempfile
from contextlib import contextmanager
from importlib import import_module
from typing import Any


def _xacro_cache_dir() -> str:
    cache_root = os.path.expanduser(
        os.environ.get(
            "ROBOT_DESCRIPTIONS_CACHE",
            "~/.cache/robot_descriptions",
        )
    )
    return os.path.join(cache_root, "xacrodoc")


def _cache_key(
    module: Any,
    xacrodoc_module: Any,
    *,
    xacro_args: dict[str, str],
) -> str:
    payload = {
        "description_name": module.__name__.split(".")[-1],
        "xacro_path": module.XACRO_PATH,
        "xacro_args": xacro_args,
        "xacrodoc_version": getattr(xacrodoc_module, "__version__", ""),
    }
    payload_json = json.dumps(payload, sort_keys=True)
    return hashlib.sha256(payload_json.encode("utf-8")).hexdigest()[:16]


@contextmanager
def _pushd(path: str):
    cwd = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(cwd)


def _generate_urdf_path(
    module: Any,
    xacrodoc_module: Any,
    *,
    xacro_args: dict[str, str],
) -> str:
    if not os.path.exists(module.XACRO_PATH):
        raise FileNotFoundError(
            f"Xacro path {module.XACRO_PATH} does not exist "
            f"in {module.__name__}"
        )

    # The Xacro may use `$(find pkg)` to reference
    # files from other ROS packages. Register explicit package paths so
    # xacrodoc can resolve those references without a ROS workspace.
    package_paths = getattr(module, "XACRO_PACKAGE_PATHS", {})
    if package_paths:
        packages_module = import_module("xacrodoc.packages")
        packages_module.update_package_cache(package_paths)

    description_name = module.__name__.split(".")[-1]
    output_dir = os.path.join(_xacro_cache_dir(), description_name)
    os.makedirs(output_dir, exist_ok=True)
    key = _cache_key(
        module,
        xacrodoc_module,
        xacro_args=xacro_args,
    )
    output_path = os.path.join(output_dir, f"{description_name}-{key}.urdf")
    if os.path.exists(output_path):
        return output_path

    xacro_dir = os.path.dirname(module.XACRO_PATH)
    # xacro might be using relative paths to refer to other macros.
    # We'll run from the file dir so we can build, then rewrite any
    # relative paths that are in the output in the next step.
    with _pushd(xacro_dir):
        doc = xacrodoc_module.XacroDoc.from_file(
            module.XACRO_PATH,
            subargs=xacro_args,
        )
    # We're resolving relative paths manually here, as xacrodoc
    # only handles package resolution. xacrodoc has a private
    # helper which would atleast make this cleaner,
    # _urdf_elements_with_filenames, but we'll wait for a
    # public interface.

    for tag_name in ("mesh", "material"):
        for elem in doc.dom.getElementsByTagName(tag_name):
            if not elem.hasAttribute("filename"):
                continue
            filename = elem.getAttribute("filename")
            rel_path = None
            if filename.startswith("file://./"):
                rel_path = filename[len("file://./") :]
            elif filename.startswith("./"):
                rel_path = filename[2:]
            if rel_path is not None:
                abs_path = os.path.abspath(
                    os.path.join(module.PACKAGE_PATH, rel_path)
                )
                elem.setAttribute("filename", abs_path)

    tmp_file = tempfile.NamedTemporaryFile(
        prefix=f"{description_name}-",
        suffix=".urdf",
        dir=output_dir,
        delete=False,
    )
    tmp_file.close()
    try:
        doc.to_urdf_file(tmp_file.name)
        os.replace(tmp_file.name, output_path)
    finally:
        if os.path.exists(tmp_file.name):
            os.unlink(tmp_file.name)

    return output_path


def get_urdf_path(
    module: Any,
    xacro_args: dict[str, str] | None = None,
) -> str:
    """Get the URDF path from a description module.

    If the module exposes `URDF_PATH`, this path is returned
    directly. If the module instead exposes `XACRO_PATH`,
    the Xacro source is rendered to a cached URDF file and the
    path to the generated file is returned.
    """
    if xacro_args is not None and not hasattr(module, "XACRO_PATH"):
        raise ValueError(
            f"{module.__name__} is not a Xacro description, "
            "so xacro_args cannot be provided"
        )

    if hasattr(module, "URDF_PATH"):
        return module.URDF_PATH

    if not hasattr(module, "XACRO_PATH"):
        raise ValueError(f"{module.__name__} has no URDF_PATH or XACRO_PATH")

    try:
        xacrodoc_module = import_module("xacrodoc")
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "This robot description requires 'xacrodoc'. "
            "Install it with `pip install xacrodoc`."
        ) from exc

    module_xacro_args = getattr(module, "XACRO_ARGS", {})
    if not isinstance(module_xacro_args, dict):
        raise TypeError("XACRO_ARGS should be a dictionary")
    effective_xacro_args = (
        module_xacro_args
        if xacro_args is None
        else {**module_xacro_args, **xacro_args}
    )

    return _generate_urdf_path(
        module,
        xacrodoc_module,
        xacro_args=effective_xacro_args,
    )
