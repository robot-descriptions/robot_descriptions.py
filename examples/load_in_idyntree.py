#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Giulio Romualdi

"""
Load a robot description selected from the command line in iDynTree.

This example requires iDynTree, which can be installed by
`conda install -c conda-forge idyntree`.
"""

import argparse

from robot_descriptions.loaders.idyntree import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    print(f"Robot successfully loaded as {robot}")
