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

    def __init__(self, single_format):
        """Initialize a description that provides a single format.

        Args:
            single_format: Format provided by the description.
        """
        self.formats = {single_format}

    @property
    def has_mjcf(self) -> bool:
        """Check if description provides MJCF."""
        return Format.MJCF in self.formats

    @property
    def has_urdf(self) -> bool:
        """Check if description provides URDF."""
        return Format.URDF in self.formats


DESCRIPTIONS: Dict[str, Description] = {
    "a1_description": Description(Format.URDF),
    "a1_mj_description": Description(Format.MJCF),
    "aliengo_description": Description(Format.URDF),
    "aliengo_mj_description": Description(Format.MJCF),
    "aloha_mj_description": Description(Format.MJCF),
    "allegro_hand_description": Description(Format.URDF),
    "allegro_hand_mj_description": Description(Format.MJCF),
    "anymal_b_description": Description(Format.URDF),
    "anymal_b_mj_description": Description(Format.MJCF),
    "anymal_c_description": Description(Format.URDF),
    "anymal_c_mj_description": Description(Format.MJCF),
    "anymal_d_description": Description(Format.URDF),
    "atlas_drc_description": Description(Format.URDF),
    "atlas_v4_description": Description(Format.URDF),
    "b1_description": Description(Format.URDF),
    "b2_description": Description(Format.URDF),
    "barrett_hand_description": Description(Format.URDF),
    "baxter_description": Description(Format.URDF),
    "berkeley_humanoid_description": Description(Format.URDF),
    "bolt_description": Description(Format.URDF),
    "cassie_description": Description(Format.URDF),
    "cassie_mj_description": Description(Format.MJCF),
    "cf2_description": Description(Format.URDF),
    "cf2_mj_description": Description(Format.MJCF),
    "double_pendulum_description": Description(Format.URDF),
    "draco3_description": Description(Format.URDF),
    "edo_description": Description(Format.URDF),
    "ergocub_description": Description(Format.URDF),
    "eve_r3_description": Description(Format.URDF),
    "fanuc_m710ic_description": Description(Format.URDF),
    "fetch_description": Description(Format.URDF),
    "finger_edu_description": Description(Format.URDF),
    "g1_mj_description": Description(Format.MJCF),
    "gen2_description": Description(Format.URDF),
    "gen3_description": Description(Format.URDF),
    "gen3_mj_description": Description(Format.MJCF),
    "ginger_description": Description(Format.URDF),
    "go1_description": Description(Format.URDF),
    "go1_mj_description": Description(Format.MJCF),
    "g1_description": Description(Format.URDF),
    "h1_description": Description(Format.URDF),
    "h1_mj_description": Description(Format.MJCF),
    "hyq_description": Description(Format.URDF),
    "icub_description": Description(Format.URDF),
    "iiwa7_description": Description(Format.URDF),
    "iiwa14_description": Description(Format.URDF),
    "iiwa14_mj_description": Description(Format.MJCF),
    "jaxon_description": Description(Format.URDF),
    "jvrc_description": Description(Format.URDF),
    "jvrc_mj_description": Description(Format.MJCF),
    "laikago_description": Description(Format.URDF),
    "leap_hand_v1": Description(Format.URDF),
    "leap_hand_mj_description": Description(Format.MJCF),
    "mini_cheetah_description": Description(Format.URDF),
    "minitaur_description": Description(Format.URDF),
    "nextage_description": Description(Format.URDF),
    "op3_mj_description": Description(Format.MJCF),
    "panda_description": Description(Format.URDF),
    "panda_mj_description": Description(Format.MJCF),
    "pepper_description": Description(Format.URDF),
    "piper_description": Description(Format.URDF),
    "piper_mj_description": Description(Format.MJCF),
    "poppy_ergo_jr_description": Description(Format.URDF),
    "poppy_torso_description": Description(Format.URDF),
    "pr2_description": Description(Format.URDF),
    "r2_description": Description(Format.URDF),
    "reachy_description": Description(Format.URDF),
    "rhea_description": Description(Format.URDF),
    "robotiq_2f85_description": Description(Format.URDF),
    "robotiq_2f85_mj_description": Description(Format.MJCF),
    "robotiq_2f85_v4_mj_description": Description(Format.MJCF),
    "romeo_description": Description(Format.URDF),
    "rsk_description": Description(Format.URDF),
    "sawyer_mj_description": Description(Format.MJCF),
    "shadow_hand_mj_description": Description(Format.MJCF),
    "simple_humanoid_description": Description(Format.URDF),
    "skydio_x2_description": Description(Format.URDF),
    "skydio_x2_mj_description": Description(Format.MJCF),
    "so_arm100": Description(Format.URDF),
    "so_arm100_mj_description": Description(Format.MJCF),
    "solo_description": Description(Format.URDF),
    "spryped_description": Description(Format.URDF),
    "stretch_description": Description(Format.URDF),
    "stretch_mj_description": Description(Format.MJCF),
    "talos_description": Description(Format.URDF),
    "talos_mj_description": Description(Format.MJCF),
    "tiago_description": Description(Format.URDF),
    "trifinger_edu_description": Description(Format.URDF),
    "upkie_description": Description(Format.URDF),
    "ur10_description": Description(Format.URDF),
    "ur10e_mj_description": Description(Format.MJCF),
    "ur3_description": Description(Format.URDF),
    "ur5_description": Description(Format.URDF),
    "ur5e_mj_description": Description(Format.MJCF),
    "valkyrie_description": Description(Format.URDF),
    "viper_mj_description": Description(Format.MJCF),
    "widow_mj_description": Description(Format.MJCF),
    "xarm7_mj_description": Description(Format.MJCF),
    "yumi_description": Description(Format.URDF),
    "z1_description": Description(Format.URDF),
    "z1_mj_description": Description(Format.MJCF),
}
