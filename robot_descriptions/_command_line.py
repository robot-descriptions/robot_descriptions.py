#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
List or show robot descriptions from the command line.

Note:
    This tool requires `yourdfpy` which is an optional dependency. It can be
    installed separately (``pip install yourdfpy``), or when robot descriptions
    are installed via ``pip install robot_descriptions[cli]``.
"""

import argparse
import sys
from functools import partial
from importlib import import_module  # type: ignore
from typing import List

import yourdfpy  # pylint: disable=import-error
from yourdfpy.viz import (  # pylint: disable=import-error
    generate_joint_limit_trajectory,
    viewer_callback,
)

from robot_descriptions._description_names import DESCRIPTION_NAMES


def positive_float(value) -> float:
    """
    Convert a value to float and check that it is positive.

    Args:
        value: Value to convert.

    Returns:
        Converted value.

    Raises:
        ArgumentTypeError: if the value is not positive.
    """
    float_value = float(value)
    if float_value <= 0.0:
        raise argparse.ArgumentTypeError(
            f"Duration {value} is not a positive number"
        )
    return float_value


def get_argument_parser() -> argparse.ArgumentParser:
    """
    Parse command-line arguments.

    Returns:
        Argument parser.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(title="subcommands", dest="subcmd")

    # list --------------------------------------------------------------------
    subparsers.add_parser(
        "list",
        help="list all available robot descriptions",
    )

    # show --------------------------------------------------------------------
    parser_show = subparsers.add_parser(
        "show",
        help="load and display a given robot description",
    )
    parser_show.add_argument(
        "name",
        help="name of the robot description",
    )
    parser_show.add_argument(
        "-c",
        "--configuration",
        nargs="+",
        type=float,
        help="configuration of the visualized robot description",
    )
    parser_show.add_argument(
        "--collision",
        action="store_true",
        help="use collision geometry for the visualized robot description",
    )

    # animate -----------------------------------------------------------------
    parser_animate = subparsers.add_parser(
        "animate",
        help=(
            "animate a robot description by interpolating "
            "each joint between its limits",
        ),
    )
    parser_animate.add_argument(
        "name",
        help="name of the robot description",
    )
    parser_animate.add_argument(
        "--collision",
        action="store_true",
        help="use collision geometry for the visualized robot description",
    )
    parser_animate.add_argument(
        "--loop-time",
        type=positive_float,
        default=6.0,
        help="duration of animation loop in seconds",
    )

    return parser


def show(
    name: str,
    configuration: List[float],
    collision: bool,
) -> None:
    """
    Load and display a given robot description.

    Args:
        name: Name of the robot description.
        configuration: Optional robot configuration.
        collision: Use collision rather than visualization geometry.
    """
    try:
        module = import_module(f"robot_descriptions.{name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{name}_description")

    if collision:
        robot = yourdfpy.URDF.load(
            module.URDF_PATH,
            build_collision_scene_graph=True,
            load_collision_meshes=True,
        )
    else:
        robot = yourdfpy.URDF.load(module.URDF_PATH)

    if configuration:
        robot.update_cfg(configuration)
    robot.show(
        collision_geometry=collision,
    )


def animate(
    name: str,
    collision: bool,
    loop_time: float,
) -> None:
    """
    Load and animate a given robot description.

    Args:
        name: Name of the robot description.
        collision: Use collision rather than visualization geometry.
        loop_time: Animation duration in seconds.
    """
    try:
        module = import_module(f"robot_descriptions.{name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{name}_description")

    if collision:
        robot = yourdfpy.URDF.load(
            module.URDF_PATH,
            build_collision_scene_graph=True,
            load_collision_meshes=True,
        )
    else:
        robot = yourdfpy.URDF.load(module.URDF_PATH)

    robot.show(
        collision_geometry=collision,
        callback=partial(
            viewer_callback,
            urdf_model=robot,
            loop_time=loop_time,
            trajectory=generate_joint_limit_trajectory(
                urdf_model=robot, loop_time=loop_time
            ),
        ),
    )


def main(argv=None):
    """
    Command line entry point.
    """
    parser = get_argument_parser()
    args = parser.parse_args(argv)
    if args.subcmd is None:
        parser.print_help()
        sys.exit(-1)

    if args.subcmd == "list":
        print("\n".join(DESCRIPTION_NAMES))
    elif args.subcmd == "show":
        show(args.name, args.configuration, args.collision)
    elif args.subcmd == "animate":
        animate(args.name, args.collision, args.loop_time)
