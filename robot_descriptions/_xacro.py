#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Inria

"""Utilities to resolve URDF files from Xacro sources."""

import hashlib
import json
import os
import re
import tempfile
from contextlib import contextmanager
from importlib import import_module
from typing import Any
from urllib.parse import unquote, urlparse


def _xacro_cache_dir() -> str:
    cache_root = os.path.expanduser(
        os.environ.get(
            "ROBOT_DESCRIPTIONS_CACHE",
            "~/.cache/robot_descriptions",
        )
    )
    return os.path.join(cache_root, "xacrodoc")


def _cache_key(module: Any, xacrodoc_module: Any) -> str:
    payload = {
        "description_name": module.__name__.split(".")[-1],
        "xacro_path": module.XACRO_PATH,
        "xacro_args": getattr(module, "XACRO_ARGS", {}),
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


def _to_package_uri(
    filename: str,
    *,
    package_name: str,
    repo_path: str,
    package_path: str,
    package_uri_root: str,
) -> str:
    """Convert a repo-local file URI to a package URI.

    Args:
        filename: Filename from a generated URDF.
        package_name: Name to use in the rewritten package URI.
        repo_path: Root path of the cloned description repository.
        package_path: Package path exported by the description module.
        package_uri_root: Root path to strip when forming the package URI.

    Returns:
        Original filename if no rewrite applies, otherwise a package URI.
    """
    if not filename.startswith("file://"):
        return filename

    parsed = urlparse(filename)
    if parsed.netloc not in ("", "localhost"):
        return filename

    abs_path = os.path.normpath(unquote(parsed.path))
    if abs_path == package_uri_root or abs_path.startswith(
        package_uri_root + os.sep
    ):
        rel_path = os.path.relpath(abs_path, package_uri_root)
        return f"package://{package_name}/{rel_path}"
    if abs_path == repo_path or abs_path.startswith(repo_path + os.sep):
        rel_path = os.path.relpath(abs_path, repo_path)
        return f"package://{package_name}/{rel_path}"
    if abs_path == package_path or abs_path.startswith(package_path + os.sep):
        rel_path = os.path.relpath(abs_path, package_path)
        return f"package://{package_name}/{rel_path}"
    return filename


def _convert_filenames_to_package_uris(module: Any, urdf_string: str) -> str:
    """Rewrite repo-local file URIs in a URDF string to package URIs.

    Args:
        module: Robot description module providing path metadata.
        urdf_string: Serialized URDF XML.

    Returns:
        URDF XML with repo-local file URIs rewritten to package URIs.
    """
    # PACKAGE_URI_NAME only overrides the visible package://<name>/ prefix.
    package_uri_name = getattr(module, "PACKAGE_URI_NAME", None)
    package_name = (
        package_uri_name
        if package_uri_name is not None
        else os.path.basename(os.path.normpath(module.REPOSITORY_PATH))
    )
    repo_path = os.path.normpath(module.REPOSITORY_PATH)
    package_path = os.path.normpath(module.PACKAGE_PATH)
    # By default, asset paths are made relative to PACKAGE_PATH. Nested
    # package layouts can override this with PACKAGE_URI_ROOT.
    package_uri_root = os.path.normpath(
        getattr(module, "PACKAGE_URI_ROOT", package_path)
    )

    def rewrite(match: re.Match[str]) -> str:
        prefix = match.group(1)
        quote = match.group(2)
        filename = match.group(3)
        new_filename = _to_package_uri(
            filename,
            package_name=package_name,
            repo_path=repo_path,
            package_path=package_path,
            package_uri_root=package_uri_root,
        )
        return f"{prefix}{quote}{new_filename}{quote}"

    return re.sub(r"(\bfilename\s*=\s*)(['\"])(.*?)\2", rewrite, urdf_string)


def _generate_urdf_path(module: Any, xacrodoc_module: Any) -> str:
    if not os.path.exists(module.XACRO_PATH):
        raise FileNotFoundError(
            f"Xacro path {module.XACRO_PATH} does not exist "
            f"in {module.__name__}"
        )

    description_name = module.__name__.split(".")[-1]
    output_dir = os.path.join(_xacro_cache_dir(), description_name)
    os.makedirs(output_dir, exist_ok=True)
    key = _cache_key(module, xacrodoc_module)
    output_path = os.path.join(output_dir, f"{description_name}-{key}.urdf")
    if os.path.exists(output_path):
        return output_path

    xacro_args = getattr(module, "XACRO_ARGS", {})
    if not isinstance(xacro_args, dict):
        raise TypeError("XACRO_ARGS should be a dictionary")

    xacro_dir = os.path.dirname(module.XACRO_PATH)
    # xacro might be using relative paths to refer to other macros.
    # We'll run from the file dir so we can build, then rewrite any
    # relative paths that are in the output in the next step.
    with _pushd(xacro_dir):
        doc = xacrodoc_module.XacroDoc.from_file(
            module.XACRO_PATH,
            subargs=xacro_args,
            # Preserve file/relative paths from the generated URDF so we can
            # normalize them to a relocatable package URI later.
            resolve_packages=False,
        )
    doc.rootdir = module.PACKAGE_PATH
    tmp_file = tempfile.NamedTemporaryFile(
        prefix=f"{description_name}-",
        suffix=".urdf",
        dir=output_dir,
        delete=False,
    )
    tmp_file.close()
    try:
        # First DOM rewrite: make all plain filenames
        # into filename:// absolute paths.
        urdf_string = doc.to_urdf_string(
            use_protocols=True,
            pretty=True,
        )
        # Second DOM rewrite: convert filename:///absolute/paths to
        # be package:// paths from the base of the cache.
        urdf_string = _convert_filenames_to_package_uris(module, urdf_string)
        with open(tmp_file.name, "w", encoding="utf-8") as urdf_file:
            urdf_file.write(urdf_string)
        os.replace(tmp_file.name, output_path)
    finally:
        if os.path.exists(tmp_file.name):
            os.unlink(tmp_file.name)

    return output_path


def get_urdf_path(module: Any) -> str:
    """Get the URDF path from a description module.

    If the module exposes `URDF_PATH`, this path is returned
    directly. If the module instead exposes `XACRO_PATH`,
    the Xacro source is rendered to a cached URDF file and the
    path to the generated file is returned.
    """
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

    return _generate_urdf_path(module, xacrodoc_module)
