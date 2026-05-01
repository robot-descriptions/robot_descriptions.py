#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Inria

"""Utilities to resolve XML description files from Xacro sources."""

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
    output_format: str,
    xacro_path: str,
    xacro_args: dict[str, str],
) -> str:
    payload = {
        "description_name": module.__name__.split(".")[-1],
        "output_format": output_format,
        "xacro_path": xacro_path,
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


def _generate_xacro_output_path(
    module: Any,
    xacrodoc_module: Any,
    *,
    output_format: str,
    xacro_path: str,
    xacro_args: dict[str, str],
) -> str:
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(
            f"Xacro path {xacro_path} does not exist in {module.__name__}"
        )

    description_name = module.__name__.split(".")[-1]
    output_dir = os.path.join(_xacro_cache_dir(), description_name)
    os.makedirs(output_dir, exist_ok=True)
    key = _cache_key(
        module,
        xacrodoc_module,
        output_format=output_format,
        xacro_path=xacro_path,
        xacro_args=xacro_args,
    )
    output_path = os.path.join(
        output_dir,
        f"{description_name}-{key}.{output_format}",
    )
    if os.path.exists(output_path):
        return output_path

    xacro_dir = os.path.dirname(xacro_path)
    # xacro might be using relative paths to refer to other macros.
    # We'll run from the file dir so we can build, then rewrite any
    # relative paths that are in the output in the next step.
    with _pushd(xacro_dir):
        doc = xacrodoc_module.XacroDoc.from_file(
            xacro_path,
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
        suffix=f".{output_format}",
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


def _get_description_path(
    module: Any,
    *,
    output_format: str,
    output_path_attr: str,
    xacro_path_attr: str,
    xacro_args_attr: str,
    xacro_args: dict[str, str] | None,
) -> str:
    if xacro_args is not None and not hasattr(module, xacro_path_attr):
        raise ValueError(
            f"{module.__name__} is not a Xacro description, "
            "so xacro_args cannot be provided"
        )

    if hasattr(module, output_path_attr):
        return getattr(module, output_path_attr)

    if not hasattr(module, xacro_path_attr):
        raise ValueError(
            f"{module.__name__} has no {output_path_attr} or "
            f"{xacro_path_attr}"
        )

    try:
        xacrodoc_module = import_module("xacrodoc")
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "This robot description requires 'xacrodoc'. "
            "Install it with `pip install xacrodoc`."
        ) from exc

    module_xacro_args = getattr(module, xacro_args_attr, {})
    if not isinstance(module_xacro_args, dict):
        raise TypeError(f"{xacro_args_attr} should be a dictionary")
    effective_xacro_args = (
        module_xacro_args
        if xacro_args is None
        else {**module_xacro_args, **xacro_args}
    )

    return _generate_xacro_output_path(
        module,
        xacrodoc_module,
        output_format=output_format,
        xacro_path=getattr(module, xacro_path_attr),
        xacro_args=effective_xacro_args,
    )


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
    return _get_description_path(
        module,
        output_format="urdf",
        output_path_attr="URDF_PATH",
        xacro_path_attr="XACRO_PATH",
        xacro_args_attr="XACRO_ARGS",
        xacro_args=xacro_args,
    )


def get_srdf_path(
    module: Any,
    xacro_args: dict[str, str] | None = None,
) -> str:
    """Get the SRDF path from a description module.

    If the module exposes `SRDF_PATH`, this path is returned
    directly. If the module instead exposes `SRDF_XACRO_PATH`,
    the Xacro source is rendered to a cached SRDF file and the
    path to the generated file is returned.
    """
    return _get_description_path(
        module,
        output_format="srdf",
        output_path_attr="SRDF_PATH",
        xacro_path_attr="SRDF_XACRO_PATH",
        xacro_args_attr="SRDF_XACRO_ARGS",
        xacro_args=xacro_args,
    )
