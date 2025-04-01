#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""List of all robot descriptions."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Set


class Format(IntEnum):
    """Format of a robot description."""

    URDF = 0
    MJCF = 1


@dataclass
class Description:
    """Metadata for a robot description.

    Attributes:
        formats: List of formats (URDF, MJCF) provided by the description.
    """

    formats: Set[Format]
    tags: Set[str]

    def __init__(self, single_format: Format, tags: Set[str] = set()):
        """Initialize a description that provides a single format.

        Args:
            single_format: Format provided by the description.
            tags: Set of strings describing properties of the description.
        """
        self.formats = {single_format}
        self.tags = tags

    @property
    def has_mjcf(self) -> bool:
        """Check if description provides MJCF."""
        return Format.MJCF in self.formats

    @property
    def has_urdf(self) -> bool:
        """Check if description provides URDF."""
        return Format.URDF in self.formats


DESCRIPTIONS: Dict[str, Description] = {
    "a1_description": Description(Format.URDF, tags={"quadruped"}),
    "a1_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "ability_hand_description": Description(
        Format.URDF, tags={"end_effector"}
    ),
    "ability_hand_mj_description": Description(
        Format.MJCF, tags={"end_effector"}
    ),
    "aliengo_description": Description(Format.URDF, tags={"quadruped"}),
    "aliengo_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "allegro_hand_description": Description(
        Format.URDF, tags={"end_effector"}
    ),
    "allegro_hand_mj_description": Description(
        Format.MJCF, tags={"end_effector"}
    ),
    "aloha_mj_description": Description(Format.MJCF, tags={"dual_arm"}),
    "anymal_b_description": Description(Format.URDF, tags={"quadruped"}),
    "anymal_b_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "anymal_c_description": Description(Format.URDF, tags={"quadruped"}),
    "anymal_c_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "anymal_d_description": Description(Format.URDF, tags={"quadruped"}),
    "apollo_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "atlas_drc_description": Description(Format.URDF, tags={"humanoid"}),
    "atlas_v4_description": Description(Format.URDF, tags={"humanoid"}),
    "b1_description": Description(Format.URDF, tags={"quadruped"}),
    "b2_description": Description(Format.URDF, tags={"quadruped"}),
    "barrett_hand_description": Description(Format.URDF),
    "baxter_description": Description(Format.URDF, tags={"dual_arm"}),
    "berkeley_humanoid_description": Description(
        Format.URDF, tags={"humanoid"}
    ),
    "bolt_description": Description(Format.URDF),
    "cassie_description": Description(Format.URDF),
    "cassie_mj_description": Description(Format.MJCF),
    "cf2_description": Description(Format.URDF),
    "cf2_mj_description": Description(Format.MJCF),
    "double_pendulum_description": Description(Format.URDF),
    "draco3_description": Description(Format.URDF, tags={"humanoid"}),
    "edo_description": Description(Format.URDF, tags={"arm"}),
    "elf2_description": Description(Format.URDF, tags={"humanoid"}),
    "elf2_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "ergocub_description": Description(Format.URDF, tags={"humanoid"}),
    "eve_r3_description": Description(Format.URDF),
    "fanuc_m710ic_description": Description(Format.URDF, tags={"arm"}),
    "fetch_description": Description(Format.URDF),
    "finger_edu_description": Description(Format.URDF),
    "g1_description": Description(Format.URDF, tags={"humanoid"}),
    "g1_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "gen2_description": Description(Format.URDF),
    "gen3_description": Description(Format.URDF),
    "gen3_mj_description": Description(Format.MJCF),
    "ginger_description": Description(Format.URDF),
    "go1_description": Description(Format.URDF, tags={"quadruped"}),
    "go1_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "go2_description": Description(Format.URDF, tags={"quadruped"}),
    "go2_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "h1_description": Description(Format.URDF, tags={"humanoid"}),
    "h1_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "hyq_description": Description(Format.URDF, tags={"quadruped"}),
    "icub_description": Description(Format.URDF, tags={"humanoid"}),
    "iiwa7_description": Description(Format.URDF),
    "iiwa14_description": Description(Format.URDF),
    "iiwa14_mj_description": Description(Format.MJCF),
    "jaxon_description": Description(Format.URDF, tags={"humanoid"}),
    "jvrc_description": Description(Format.URDF, tags={"humanoid"}),
    "jvrc_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "laikago_description": Description(Format.URDF, tags={"quadruped"}),
    "leap_hand_v1": Description(Format.URDF),
    "leap_hand_mj_description": Description(Format.MJCF),
    "mini_cheetah_description": Description(Format.URDF, tags={"quadruped"}),
    "minitaur_description": Description(Format.URDF, tags={"quadruped"}),
    "nextage_description": Description(Format.URDF, tags={"dual_arm"}),
    "op3_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "panda_description": Description(Format.URDF),
    "panda_mj_description": Description(Format.MJCF),
    "pepper_description": Description(Format.URDF),
    "piper_description": Description(Format.URDF),
    "piper_mj_description": Description(Format.MJCF),
    "poppy_ergo_jr_description": Description(Format.URDF),
    "poppy_torso_description": Description(Format.URDF, tags={"dual_arm"}),
    "pr2_description": Description(Format.URDF),
    "r2_description": Description(Format.URDF, tags={"humanoid"}),
    "reachy_description": Description(Format.URDF),
    "rhea_description": Description(Format.URDF),
    "robotiq_2f85_description": Description(Format.URDF),
    "robotiq_2f85_mj_description": Description(Format.MJCF),
    "robotiq_2f85_v4_mj_description": Description(Format.MJCF),
    "romeo_description": Description(Format.URDF, tags={"humanoid"}),
    "rsk_description": Description(Format.URDF),
    "sawyer_mj_description": Description(Format.MJCF),
    "shadow_hand_mj_description": Description(Format.MJCF),
    "sigmaban_description": Description(Format.URDF, tags={"humanoid"}),
    "simple_humanoid_description": Description(Format.URDF, tags={"humanoid"}),
    "skydio_x2_description": Description(Format.URDF),
    "skydio_x2_mj_description": Description(Format.MJCF),
    "so_arm100": Description(Format.URDF),
    "so_arm100_mj_description": Description(Format.MJCF),
    "solo_description": Description(Format.URDF, tags={"quadruped"}),
    "spot_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "spryped_description": Description(Format.URDF),
    "stretch_description": Description(Format.URDF),
    "stretch_mj_description": Description(Format.MJCF),
    "talos_description": Description(Format.URDF, tags={"humanoid"}),
    "talos_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "tiago_description": Description(Format.URDF),
    "trifinger_edu_description": Description(Format.URDF),
    "upkie_description": Description(Format.URDF),
    "ur10_description": Description(Format.URDF),
    "ur10e_mj_description": Description(Format.MJCF),
    "ur3_description": Description(Format.URDF),
    "ur5_description": Description(Format.URDF),
    "ur5e_mj_description": Description(Format.MJCF),
    "valkyrie_description": Description(Format.URDF, tags={"humanoid"}),
    "viper_mj_description": Description(Format.MJCF),
    "widow_mj_description": Description(Format.MJCF),
    "xarm7_mj_description": Description(Format.MJCF),
    "yumi_description": Description(Format.URDF, tags={"dual_arm"}),
    "z1_description": Description(Format.URDF),
    "z1_mj_description": Description(Format.MJCF),
}
