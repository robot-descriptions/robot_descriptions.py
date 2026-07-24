#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Utilities to resolve XML description files from Xacro sources."""

import hashlib
import json
import os
import re
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass
from importlib import import_module
from typing import Any
from urllib.parse import unquote, urlparse

from ._cache import get_head_sha


@dataclass(frozen=True)
class DescriptionFormatAttrs:
    """Names of the module attributes exposing a description in a format.

    A description module advertises each format it supports through a
    small set of module-level attributes. This dataclass records which
    attribute names to look up for a given format.

    Attributes:
        path_attr: Name of the module attribute holding a ready-to-use
            path to the description file, e.g. ``"URDF_PATH"`` for URDF.
        xacro_path_attr: Optional name of the module attribute holding
            the path to a Xacro template rendered to produce the
            description, e.g. ``"XACRO_PATH"``, or ``None`` for
            non-templated formats like MJCF.
        xacro_args_attr: Optional name of the module attribute holding
            the default Xacro arguments (a dictionary) passed when
            rendering a Xacro source, or ``None`` for non-templated
            formats.
    """

    path_attr: str
    xacro_path_attr: str | None = None
    xacro_args_attr: str | None = None


_DESCRIPTION_FORMAT_ATTRS = {
    "urdf": DescriptionFormatAttrs("URDF_PATH", "XACRO_PATH", "XACRO_ARGS"),
    "mjcf": DescriptionFormatAttrs("MJCF_PATH"),
    "srdf": DescriptionFormatAttrs(
        "SRDF_PATH", "SRDF_XACRO_PATH", "SRDF_XACRO_ARGS"
    ),
}


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
    package_commits: dict[str, str],
) -> str:
    payload = {
        "description_name": module.__name__.split(".")[-1],
        "output_format": output_format,
        "xacro_path": xacro_path,
        "xacro_args": xacro_args,
        "xacrodoc_version": getattr(xacrodoc_module, "__version__", ""),
        "package_commits": package_commits,
    }
    payload_json = json.dumps(payload, sort_keys=True)
    return hashlib.sha256(payload_json.encode("utf-8")).hexdigest()[:16]


def _resolve_package_commits(
    module: Any,
    package_paths: dict[str, str],
) -> dict[str, str]:
    description_name = module.__name__.split(".")[-1]
    paths_to_hash = {description_name: module.PACKAGE_PATH, **package_paths}
    commits = {}
    for name, path in paths_to_hash.items():
        sha = get_head_sha(path)
        if sha is not None:
            commits[name] = sha
    return commits


@contextmanager
def _pushd(path: str):
    cwd = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(cwd)


_FILENAME_ATTR_RE = re.compile(r"(\bfilename\s*=\s*)(['\"])(.*?)\2")


def _map_filename_attrs(urdf_string: str, transform) -> str:
    """Rewrite each ``filename="..."`` attribute value via a transform.

    Args:
        urdf_string: Serialized URDF XML.
        transform: Callable mapping a filename value to its replacement.

    Returns:
        URDF XML with each filename attribute rewritten.
    """

    def rewrite(match: re.Match[str]) -> str:
        prefix, quote, filename = match.groups()
        return f"{prefix}{quote}{transform(filename)}{quote}"

    return _FILENAME_ATTR_RE.sub(rewrite, urdf_string)


def _to_package_uri(
    filename: str, *, package_name: str, package_path: str
) -> str:
    """Rewrite a ``file://`` URI pointing inside the package to a package URI.

    A ``file://`` reference to a file under ``package_path`` becomes
    ``package://<package_name>/<path relative to package_path>``. Anything
    else -- a non-``file`` URI, a remote host, or a path outside the package
    -- is returned unchanged.

    Args:
        filename: Filename attribute value from a generated URDF.
        package_name: Package name to use in the URI, i.e. the basename of
            ``package_path``.
        package_path: Absolute path to the package root.

    Returns:
        The rewritten ``package://`` URI, or ``filename`` unchanged when no
        rewrite applies.
    """
    if not filename.startswith("file://"):
        return filename

    parsed = urlparse(filename)
    if parsed.netloc not in ("", "localhost"):
        return filename

    abs_path = os.path.normpath(unquote(parsed.path))
    if abs_path == package_path or abs_path.startswith(package_path + os.sep):
        rel_path = os.path.relpath(abs_path, package_path)
        return f"package://{package_name}/{rel_path}"
    return filename


def _convert_filenames_to_package_uris(module: Any, urdf_string: str) -> str:
    """Rewrite package-local file URIs in a URDF string to package URIs.

    Args:
        module: Robot description module providing path metadata.
        urdf_string: Serialized URDF XML.

    Returns:
        URDF XML with package-local file URIs rewritten to package URIs.
    """
    package_path = os.path.normpath(module.PACKAGE_PATH)
    package_name = os.path.basename(package_path)

    return _map_filename_attrs(
        urdf_string,
        lambda filename: _to_package_uri(
            filename,
            package_name=package_name,
            package_path=package_path,
        ),
    )


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
    package_commits = _resolve_package_commits(module, package_paths)
    key = _cache_key(
        module,
        xacrodoc_module,
        output_format=output_format,
        xacro_path=xacro_path,
        xacro_args=xacro_args,
        package_commits=package_commits,
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
            # Preserve file/relative paths from the generated URDF so we can
            # normalize them to a relocatable package URI later.
            resolve_packages=False,
        )
    doc.rootdir = module.PACKAGE_PATH
    tmp_file = tempfile.NamedTemporaryFile(
        prefix=f"{description_name}-",
        suffix=f".{output_format}",
        dir=output_dir,
        delete=False,
    )
    tmp_file.close()
    try:
        # Serialize with protocol prefixes so local asset paths become
        # absolute file:// URIs (package:// references are kept as-is).
        urdf_string = doc.to_urdf_string(
            use_protocols=True,
            pretty=True,
        )
        # Rewrite those file:// URIs to package:// URIs relative to the
        # package, so the cached URDF is relocatable.
        urdf_string = _convert_filenames_to_package_uris(module, urdf_string)
        with open(tmp_file.name, "w", encoding="utf-8") as urdf_file:
            urdf_file.write(urdf_string)
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
    xacro_path_attr: str | None,
    xacro_args_attr: str | None,
    xacro_args: dict[str, str] | None,
) -> str:
    if xacro_args is not None and (
        xacro_path_attr is None or not hasattr(module, xacro_path_attr)
    ):
        raise ValueError(
            f"{module.__name__} is not a Xacro description, "
            "so xacro_args cannot be provided"
        )

    if hasattr(module, output_path_attr):
        return getattr(module, output_path_attr)

    if xacro_path_attr is None or not hasattr(module, xacro_path_attr):
        raise ValueError(
            f"{module.__name__} has no supported {output_format.upper()} path"
        )

    try:
        xacrodoc_module = import_module("xacrodoc")
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "This robot description requires 'xacrodoc'. "
            "Install it with `pip install xacrodoc`."
        ) from exc

    module_xacro_args = getattr(module, xacro_args_attr or "", {})
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


def has_description_path(module: Any, description_format: str) -> bool:
    """Check whether a module can resolve a description format.

    Args:
        module: Description module.
        description_format: Requested description format, e.g. "urdf"
            or "srdf".

    Returns:
        True if the description module provides the requested format.
    """
    if description_format not in _DESCRIPTION_FORMAT_ATTRS:
        return False
    attrs = _DESCRIPTION_FORMAT_ATTRS[description_format]
    return hasattr(module, attrs.path_attr) or (
        attrs.xacro_path_attr is not None
        and hasattr(module, attrs.xacro_path_attr)
    )


def get_description_path(
    module: Any,
    description_format: str,
    xacro_args: dict[str, str] | None = None,
) -> str:
    """Get a description path from a description module.

    Args:
        module: Description module.
        description_format: Format to resolve.
        xacro_args: Optional xacro arguments.

    Returns:
        Path to the description file in the requested format.
    """
    if description_format not in _DESCRIPTION_FORMAT_ATTRS:
        raise ValueError(
            f"Unsupported description format: {description_format}"
        )
    attrs = _DESCRIPTION_FORMAT_ATTRS[description_format]
    return _get_description_path(
        module,
        output_format=description_format,
        output_path_attr=attrs.path_attr,
        xacro_path_attr=attrs.xacro_path_attr,
        xacro_args_attr=attrs.xacro_args_attr,
        xacro_args=xacro_args,
    )


def get_urdf_path(
    module: Any,
    xacro_args: dict[str, str] | None = None,
) -> str:
    """Get the URDF path from a description module.

    Args:
        module: Description module.
        xacro_args: Optional xacro arguments.

    Returns:
        If the module exposes `URDF_PATH`, this path is returned
        directly. If the module instead exposes `XACRO_PATH`, the
        Xacro source is rendered to a cached URDF file and the path
        to the generated file is returned.
    """
    return get_description_path(module, "urdf", xacro_args=xacro_args)


def get_mjcf_path(
    module: Any,
) -> str:
    """Get the MJCF path from a description module.

    Args:
        module: Description module.

    Returns:
        If the module exposes `MJCF_PATH`, this path is returned directly.
    """
    return get_description_path(module, "mjcf")


def get_srdf_path(
    module: Any,
    xacro_args: dict[str, str] | None = None,
) -> str:
    """Get the SRDF path from a description module.

    Args:
        module: Description module.
        xacro_args: Optional xacro arguments.

    Returns:
        If the module exposes `SRDF_PATH`, this path is returned
        directly. If the module instead exposes `SRDF_XACRO_PATH`, the
        Xacro source is rendered to a cached SRDF file and the path to
        the generated file is returned.
    """
    return get_description_path(module, "srdf", xacro_args=xacro_args)
