#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""List or show robot descriptions from the command line."""

import argparse
from importlib import import_module  # type: ignore
from typing import List

from robot_descriptions._descriptions import DESCRIPTIONS


def positive_float(value) -> float:
    """Convert a value to float and check that it is positive.

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
    """Parse command-line arguments.

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

    # pull ----------------------------------------------------------------
    parser_pull = subparsers.add_parser(
        "pull",
        help="load given robot description from the web and save it in cache",
    )
    parser_pull.add_argument(
        "name",
        help="name of the robot description",
    )

    # show_in_meshcat --------------------------------------------------------
    parser_meshcat = subparsers.add_parser(
        "show_in_meshcat",
        help="load and display a given robot description in Meshcat",
    )
    parser_meshcat.add_argument(
        "name",
        help="name of the robot description",
    )

    # show_in_mujoco --------------------------------------------------------
    parser_mujoco = subparsers.add_parser(
        "show_in_mujoco",
        help="load and display a given robot description in mujoco",
    )
    parser_mujoco.add_argument(
        "name",
        help="name of the robot description",
    )

    # show_in_pybullet --------------------------------------------------------
    parser_pybullet = subparsers.add_parser(
        "show_in_pybullet",
        help="load and display a given robot description in pybullet",
    )
    parser_pybullet.add_argument(
        "name",
        help="name of the robot description",
    )

    # show_in_yourdfpy --------------------------------------------------------
    parser_yourdfpy = subparsers.add_parser(
        "show_in_yourdfpy",
        help="load and display a given robot description with yourdfpy",
    )
    parser_yourdfpy.add_argument(
        "name",
        help="name of the robot description",
    )
    parser_yourdfpy.add_argument(
        "-c",
        "--configuration",
        nargs="+",
        type=float,
        help="configuration of the visualized robot description",
    )
    parser_yourdfpy.add_argument(
        "--collision",
        action="store_true",
        help="use collision geometry for the visualized robot description",
    )

    # animate -----------------------------------------------------------------
    parser_animate = subparsers.add_parser(
        "animate",
        help=(
            "animate a robot description by interpolating "
            "each joint between its limits"
        ),
    )
    parser_animate.add_argument(
        "name",
        help="name of the robot description",
    )

    return parser


def list_descriptions():
    """List descriptions to the standard output."""
    for name in sorted(list(DESCRIPTIONS)):
        desc = DESCRIPTIONS[name]
        formats = ("URDF" if desc.has_urdf else "") + (
            "MJCF" if desc.has_mjcf else ""
        )
        print(f"- {name} [{formats}]")


def pull(name: str) -> None:
    """Pull a robot description from the web and save it in cache.

    Args:
        name: Name of the robot description.
    """
    try:
        module = import_module(f"robot_descriptions.{name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{name}_description")
    if hasattr(module, "URDF_PATH"):
        print(module.URDF_PATH)
    elif hasattr(module, "MJCF_PATH"):
        print(module.MJCF_PATH)


def show_in_meshcat(name: str) -> None:
    """Show a robot description in MeshCat.

    Args:
        name: Name of the robot description.
    """
    try:
        from pinocchio.visualize import MeshcatVisualizer
    except ImportError as e:
        raise ImportError(
            "Pinocchio not found, try for instance `conda install pinocchio`"
        ) from e

    from robot_descriptions.loaders.pinocchio import load_robot_description

    try:
        robot = load_robot_description(name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{name}_description")

    robot.setVisualizer(MeshcatVisualizer())
    robot.initViewer(open=True)
    robot.loadViewerModel()
    robot.display(robot.q0)

    input("Press Enter to close MeshCat and terminate... ")


def show_in_mujoco(name: str) -> None:
    """Show a robot description in MuJoCo.

    Args:
        name: Name of the robot description.
    """
    import mujoco

    try:
        import mujoco_viewer
    except ImportError as e:
        raise ImportError(
            "MuJoCo viewer not found, try `pip install mujoco-python-viewer`"
        ) from e

    from robot_descriptions.loaders.mujoco import load_robot_description

    try:
        model = load_robot_description(name)
    except ModuleNotFoundError:
        model = load_robot_description(f"{name}_mj_description")

    data = mujoco.MjData(model)
    viewer = mujoco_viewer.MujocoViewer(model, data)
    mujoco.mj_step(model, data)  # step at least once to load model in viewer
    while viewer.is_alive:
        viewer.render()
    viewer.close()


def show_in_pybullet(name: str) -> None:
    """Show a robot description in PyBullet.

    Args:
        name: Name of the robot description.
    """
    import pybullet

    from robot_descriptions.loaders.pybullet import load_robot_description

    pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

    try:
        load_robot_description(name)
    except ModuleNotFoundError:
        load_robot_description(f"{name}_description")

    input("Press Enter to close PyBullet and terminate... ")
    pybullet.disconnect()


def show_in_yourdfpy(
    name: str,
    configuration: List[float],
    collision: bool,
) -> None:
    """Load and display a given robot description.

    Args:
        name: Name of the robot description.
        configuration: Optional robot configuration.
        collision: Use collision rather than visualization geometry.
    """
    try:
        module = import_module(f"robot_descriptions.{name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{name}_description")
    if not hasattr(module, "URDF_PATH"):
        raise ValueError(
            "This command only works with URDF descriptions, use "
            "`show_in_mujoco` for MJCF descriptions"
        )

    try:
        yourdfpy = import_module("yourdfpy")
    except ModuleNotFoundError as exc:
        raise ValueError(
            "This command requires yourdfpy. You can installed it by: "
            "`pip install yourdfpy`"
        ) from exc

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


def animate(name: str) -> None:
    """Show how to call yourdfpy to animate a given robot description.

    Args:
        name: Name of the robot description.
    """
    try:
        module = import_module(f"robot_descriptions.{name}")
    except ModuleNotFoundError:
        module = import_module(f"robot_descriptions.{name}_description")

    if not hasattr(module, "URDF_PATH"):
        raise ValueError(
            "animation is only available for URDF descriptions, "
            "check out the `show_in_mujoco.py` example for MJCF"
        )

    print(
        "Check out yourdfpy more URDF features like animation or checking "
        "collision meshes. In this instance, the yourdfpy command is:\n\n"
        f"\tyourdfpy {module.URDF_PATH} --animate\n"
    )


def main(argv=None):
    """Command line entry point."""
    parser = get_argument_parser()
    args = parser.parse_args(argv)
    if args.subcmd == "list":
        list_descriptions()
    elif args.subcmd == "pull":
        pull(args.name)
    elif args.subcmd == "show_in_meshcat":
        show_in_meshcat(args.name)
    elif args.subcmd == "show_in_mujoco":
        show_in_mujoco(args.name)
    elif args.subcmd == "show_in_pybullet":
        show_in_pybullet(args.name)
    elif args.subcmd == "show_in_yourdfpy":
        show_in_yourdfpy(args.name, args.configuration, args.collision)
    elif args.subcmd == "animate":
        animate(args.name)
    else:  # no subcommand
        parser.print_help()


if __name__ == "__main__":
    main()
