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
    "barrett_hand_description": Description(
        Format.URDF, tags={"end_effector"}
    ),
    "baxter_description": Description(Format.URDF, tags={"dual_arm"}),
    "berkeley_humanoid_description": Description(
        Format.URDF, tags={"humanoid"}
    ),
    "bolt_description": Description(Format.URDF, tags={"biped"}),
    "cassie_description": Description(Format.URDF, tags={"biped"}),
    "cassie_mj_description": Description(Format.MJCF, tags={"biped"}),
    "cf2_description": Description(Format.URDF, tags={"drone"}),
    "cf2_mj_description": Description(Format.MJCF, tags={"drone"}),
    "double_pendulum_description": Description(
        Format.URDF, tags={"educational"}
    ),
    "draco3_description": Description(Format.URDF, tags={"humanoid"}),
    "edo_description": Description(Format.URDF, tags={"arm"}),
    "elf2_description": Description(Format.URDF, tags={"humanoid"}),
    "elf2_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "ergocub_description": Description(Format.URDF, tags={"humanoid"}),
    "eve_r3_description": Description(
        Format.URDF, tags={"mobile_manipulator"}
    ),
    "fanuc_m710ic_description": Description(Format.URDF, tags={"arm"}),
    "fetch_description": Description(Format.URDF, tags={"mobile_manipulator"}),
    "finger_edu_description": Description(Format.URDF, tags={"educational"}),
    "fr3_mj_description": Description(Format.MJCF, tags={"arm"}),
    "g1_description": Description(Format.URDF, tags={"humanoid"}),
    "g1_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "gen2_description": Description(Format.URDF, tags={"arm"}),
    "gen3_description": Description(Format.URDF, tags={"arm"}),
    "gen3_lite_description": Description(Format.URDF, tags={"arm"}),
    "gen3_mj_description": Description(Format.MJCF, tags={"arm"}),
    "ginger_description": Description(
        Format.URDF, tags={"mobile_manipulator"}
    ),
    "go1_description": Description(Format.URDF, tags={"quadruped"}),
    "go1_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "go2_description": Description(Format.URDF, tags={"quadruped"}),
    "go2_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "h1_description": Description(Format.URDF, tags={"humanoid"}),
    "h1_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "hyq_description": Description(Format.URDF, tags={"quadruped"}),
    "icub_description": Description(Format.URDF, tags={"humanoid"}),
    "iiwa7_description": Description(Format.URDF, tags={"arm"}),
    "iiwa14_description": Description(Format.URDF, tags={"arm"}),
    "iiwa14_mj_description": Description(Format.MJCF, tags={"arm"}),
    "jaxon_description": Description(Format.URDF, tags={"humanoid"}),
    "jvrc_description": Description(Format.URDF, tags={"humanoid"}),
    "jvrc_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "laikago_description": Description(Format.URDF, tags={"quadruped"}),
    "leap_hand_v1": Description(Format.URDF, tags={"end_effector"}),
    "leap_hand_mj_description": Description(
        Format.MJCF, tags={"end_effector"}
    ),
    "mini_cheetah_description": Description(Format.URDF, tags={"quadruped"}),
    "minitaur_description": Description(Format.URDF, tags={"quadruped"}),
    "nextage_description": Description(Format.URDF, tags={"dual_arm"}),
    "op3_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "panda_description": Description(Format.URDF, tags={"arm"}),
    "panda_mj_description": Description(Format.MJCF, tags={"arm"}),
    "pepper_description": Description(
        Format.URDF, tags={"mobile_manipulator"}
    ),
    "piper_description": Description(Format.URDF, tags={"arm"}),
    "piper_mj_description": Description(Format.MJCF, tags={"arm"}),
    "poppy_ergo_jr_description": Description(Format.URDF, tags={"arm"}),
    "poppy_torso_description": Description(Format.URDF, tags={"dual_arm"}),
    "pr2_description": Description(
        Format.URDF, tags={"dual_arm", "mobile_manipulator"}
    ),
    "r2_description": Description(Format.URDF, tags={"humanoid"}),
    "reachy_description": Description(
        Format.URDF, tags={"mobile_manipulator"}
    ),
    "rhea_description": Description(Format.URDF, tags={"biped"}),
    "robotiq_2f85_description": Description(
        Format.URDF, tags={"end_effector"}
    ),
    "robotiq_2f85_mj_description": Description(
        Format.MJCF, tags={"end_effector"}
    ),
    "robotiq_2f85_v4_mj_description": Description(
        Format.MJCF, tags={"end_effector"}
    ),
    "romeo_description": Description(Format.URDF, tags={"humanoid"}),
    "rsk_description": Description(Format.URDF, tags={"wheeled"}),
    "sawyer_mj_description": Description(Format.MJCF, tags={"arm"}),
    "shadow_dexee_mj_description": Description(
        Format.MJCF, tags={"end_effector"}
    ),
    "shadow_hand_mj_description": Description(
        Format.MJCF, tags={"end_effector"}
    ),
    "sigmaban_description": Description(Format.URDF, tags={"humanoid"}),
    "simple_humanoid_description": Description(
        Format.URDF, tags={"humanoid", "educational"}
    ),
    "skydio_x2_description": Description(Format.URDF, tags={"drone"}),
    "skydio_x2_mj_description": Description(Format.MJCF, tags={"drone"}),
    "so_arm100": Description(Format.URDF, tags={"arm"}),
    "so_arm100_mj_description": Description(Format.MJCF, tags={"arm"}),
    "solo_description": Description(Format.URDF, tags={"quadruped"}),
    "spot_mj_description": Description(Format.MJCF, tags={"quadruped"}),
    "spryped_description": Description(Format.URDF, tags={"biped"}),
    "stretch_description": Description(
        Format.URDF, tags={"mobile_manipulator"}
    ),
    "stretch_mj_description": Description(
        Format.MJCF, tags={"mobile_manipulator"}
    ),
    "stretch_3_mj_description": Description(
        Format.MJCF, tags={"mobile_manipulator"}
    ),
    "talos_description": Description(Format.URDF, tags={"humanoid"}),
    "talos_mj_description": Description(Format.MJCF, tags={"humanoid"}),
    "tiago_description": Description(Format.URDF, tags={"mobile_manipulator"}),
    "trifinger_edu_description": Description(
        Format.URDF, tags={"educational"}
    ),
    "upkie_description": Description(Format.URDF, tags={"biped", "wheeled"}),
    "ur10_description": Description(Format.URDF, tags={"arm"}),
    "ur10e_mj_description": Description(Format.MJCF, tags={"arm"}),
    "ur3_description": Description(Format.URDF, tags={"arm"}),
    "ur5_description": Description(Format.URDF, tags={"arm"}),
    "ur5e_mj_description": Description(Format.MJCF, tags={"arm"}),
    "valkyrie_description": Description(Format.URDF, tags={"humanoid"}),
    "viper_mj_description": Description(Format.MJCF, tags={"arm"}),
    "widow_mj_description": Description(Format.MJCF, tags={"arm"}),
    "xarm7_mj_description": Description(Format.MJCF, tags={"arm"}),
    "yumi_description": Description(Format.URDF, tags={"dual_arm"}),
    "z1_description": Description(Format.URDF, tags={"arm"}),
    "z1_mj_description": Description(Format.MJCF, tags={"arm"}),
}
