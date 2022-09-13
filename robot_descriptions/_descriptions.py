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
List of all robot descriptions.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Set


class Format(IntEnum):

    """
    Format of a robot description.
    """

    URDF = 0
    MJCF = 1


@dataclass
class Description:

    """
    Metadata for a robot description.

    Attributes:
        formats: List of formats (URDF, MJCF) provided by the description.
    """

    formats: Set[Format]

    def __init__(self, single_format):
        """
        Initialize a description that provides a single format.

        Args:
            single_format: Format provided by the description.
        """
        self.formats = {single_format}

    @property
    def has_mjcf(self) -> bool:
        """
        Check if description provides MJCF.
        """
        return Format.MJCF in self.formats

    @property
    def has_urdf(self) -> bool:
        """
        Check if description provides URDF.
        """
        return Format.URDF in self.formats


DESCRIPTIONS: Dict[str, Description] = {
    "a1_description": Description(Format.URDF),
    "a1_mj_description": Description(Format.MJCF),
    "aliengo_description": Description(Format.MJCF),
    "allegro_hand_description": Description(Format.URDF),
    "anymal_b_description": Description(Format.URDF),
    "anymal_b_mj_description": Description(Format.MJCF),
    "anymal_c_description": Description(Format.URDF),
    "anymal_c_mj_description": Description(Format.MJCF),
    "atlas_drc_description": Description(Format.URDF),
    "atlas_v4_description": Description(Format.URDF),
    "baxter_description": Description(Format.URDF),
    "bolt_description": Description(Format.URDF),
    "cassie_description": Description(Format.URDF),
    "cassie_mj_description": Description(Format.MJCF),
    "cf2_description": Description(Format.URDF),
    "double_pendulum_description": Description(Format.URDF),
    "edo_description": Description(Format.URDF),
    "fetch_description": Description(Format.URDF),
    "finger_edu_description": Description(Format.URDF),
    "gen2_description": Description(Format.URDF),
    "ginger_description": Description(Format.URDF),
    "go1_description": Description(Format.URDF),
    "go1_mj_description": Description(Format.MJCF),
    "hyq_description": Description(Format.URDF),
    "icub_description": Description(Format.URDF),
    "iiwa_description": Description(Format.URDF),
    "jvrc_description": Description(Format.URDF),
    "jvrc_mj_description": Description(Format.MJCF),
    "laikago_description": Description(Format.URDF),
    "mini_cheetah_description": Description(Format.URDF),
    "minitaur_description": Description(Format.URDF),
    "panda_description": Description(Format.URDF),
    "panda_mj_description": Description(Format.MJCF),
    "pepper_description": Description(Format.URDF),
    "poppy_ergo_jr_description": Description(Format.URDF),
    "poppy_torso_description": Description(Format.URDF),
    "pr2_description": Description(Format.URDF),
    "reachy_description": Description(Format.URDF),
    "robotiq_2f85_description": Description(Format.URDF),
    "robotiq_2f85_mj_description": Description(Format.MJCF),
    "romeo_description": Description(Format.URDF),
    "shadow_hand_mj_description": Description(Format.MJCF),
    "simple_humanoid_description": Description(Format.URDF),
    "solo_description": Description(Format.URDF),
    "talos_description": Description(Format.URDF),
    "tiago_description": Description(Format.URDF),
    "upkie_description": Description(Format.URDF),
    "ur10_description": Description(Format.URDF),
    "ur3_description": Description(Format.URDF),
    "ur5_description": Description(Format.URDF),
    "ur5e_mj_description": Description(Format.MJCF),
    "yumi_description": Description(Format.URDF),
}
