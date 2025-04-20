#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""Git utility functions to clone model repositories."""

from dataclasses import dataclass
from typing import Dict


@dataclass
class Repository:
    """Remote git repository.

    Attributes:
        cache_path: Path to clone the repository to in the local cache.
        commit: Commit ID or tag to checkout after cloning.
        url: URL to the remote git repository.
    """

    cache_path: str
    commit: str
    url: str


REPOSITORIES: Dict[str, Repository] = {
    "ability-hand-api": Repository(
        url="https://github.com/psyonicinc/ability-hand-api.git",
        commit="89407424edfc22faceaedcd7c3ea2b7947cbbb2c",
        cache_path="ability_hand_api",
    ),
    "anymal_b_simple_description": Repository(
        url="https://github.com/ANYbotics/anymal_b_simple_description.git",
        commit="988b5df22b84761bdf08111b1c2ccc883793f456",
        cache_path="anymal_b_simple_description",
    ),
    "anymal_c_simple_description": Repository(
        url="https://github.com/ANYbotics/anymal_c_simple_description.git",
        commit="f160b8f7fed840c47a6febe8e2bc78b32bf43a68",
        cache_path="anymal_c_simple_description",
    ),
    "anymal_d_simple_description": Repository(
        url="https://github.com/ANYbotics/anymal_d_simple_description.git",
        commit="6adc14720aab583613975e5a9d6d4fa3cfcdd081",
        cache_path="anymal_d_simple_description",
    ),
    "baxter_common": Repository(
        url="https://github.com/RethinkRobotics/baxter_common.git",
        commit="6c4b0f375fe4e356a3b12df26ef7c0d5e58df86e",  # v1.2.0
        cache_path="baxter_common",
    ),
    "berkeley_humanoid_description": Repository(
        url="https://github.com/HybridRobotics/berkeley_humanoid_description",
        commit="d0d13d3f81d795480e25ed1910eaf83d5f0a1d0b",
        cache_path="berkeley_humanoid_description",
    ),
    "bhand_model": Repository(
        url="https://github.com/jhu-lcsr-attic/bhand_model.git",
        commit="937f4186d6458bd682a7dae825fb6f4efe56ec69",
        cache_path="bhand_model",
    ),
    "bullet3": Repository(
        url="https://github.com/bulletphysics/bullet3.git",
        commit="7dee3436e747958e7088dfdcea0e4ae031ce619e",
        cache_path="bullet3",
    ),
    "bxi_robot_models": Repository(
        url="https://github.com/bxirobotics/robot_models.git",
        commit="eabe24ce937f8e633077a163b883e92e8996c36e",
        cache_path="bxi_robot_models",
    ),
    "cassie_description": Repository(
        url="https://github.com/robot-descriptions/cassie_description",
        commit="81a2d8bbd77201cc974afb127adda4e2857a6dbf",
        cache_path="cassie_description",
    ),
    "cassie_mj_description": Repository(
        url="https://github.com/rohanpsingh/cassie_mj_description.git",
        commit="fcc3775453e4da6797cd4eacd3d8321d9906755a",
        cache_path="cassie_mj_description",
    ),
    "differentiable-robot-model": Repository(
        url="https://github.com/facebookresearch/differentiable-robot-model",
        commit="d7bd1b3b8ef1d6dabe9b68474a622185c510e112",
        cache_path="differentiable-robot-model",
    ),
    "draco3_description": Repository(
        url="https://github.com/shbang91/draco3_description.git",
        commit="5afd19733d7b3e9f1135ba93e0aad90ed1a24cc7",
        cache_path="draco3_description",
    ),
    "drake": Repository(
        url="https://github.com/RobotLocomotion/drake.git",
        commit="7abea0556ede980a5077fe1a8cfbae59b57c7c27",
        cache_path="drake",
    ),
    "drake_models": Repository(
        url="https://github.com/RobotLocomotion/models.git",
        # NB: from 9b274a2570ddefc4140e4b98bcf248003289b870 onward, obj meshes
        # for the iiwa14_description are converted to glTF in this repo, which
        # PyBullet does not load ("invalid mesh filename extension 'gltf'")
        commit="d0fe1a427a6bd39040ff3a77aaf6ddcc4d62a8fe",
        cache_path="drake_models",
    ),
    "eDO_description": Repository(
        url="https://github.com/ianathompson/eDO_description.git",
        commit="17b3f92f834746106d6a4befaab8eeab3ac248e6",
        cache_path="edo_sim",  # match package name
    ),
    "ergocub-software": Repository(
        url="https://github.com/icub-tech-iit/ergocub-software.git",
        commit="v0.7.7",
        cache_path="ergocub-software",
    ),
    "example-robot-data": Repository(
        url="https://github.com/Gepetto/example-robot-data.git",
        commit="d0d9098d752014aec3725b07766962acf06c5418",  # v4.0.9
        cache_path="example-robot-data",
    ),
    "fanuc_m710ic_description": Repository(
        url="https://github.com/"
        "robot-descriptions/fanuc_m710ic_description.git",
        commit="d12af44559cd7e46f7afd513237f159f82f8402e",
        cache_path="fanuc_m710ic_description",
    ),
    "GingerURDF": Repository(
        url="https://github.com/Rayckey/GingerURDF.git",
        commit="6a1307cd0ee2b77c82f8839cdce3a2e2eed2bd8f",
        cache_path="gingerurdf",  # match package name
    ),
    "gym-pybullet-drones": Repository(
        url="https://github.com/utiasDSL/gym-pybullet-drones.git",
        commit="2c2470c2043eb4cf474d9af2681c9fa10f2d0e2f",
        cache_path="gym-pybullet-drones",
    ),
    "halodi-robot-models": Repository(
        url="https://github.com/Halodi/halodi-robot-models.git",
        commit="ba9e7c8cdbd63e20fc6526dbbea1b91c102fb820",
        cache_path="halodi-robot-models",
    ),
    "icub-models": Repository(
        url="https://github.com/robotology/icub-models.git",
        commit="v1.25.0",
        cache_path="icub-models",
    ),
    "jaxon_description": Repository(
        url="https://github.com/robot-descriptions/jaxon_description.git",
        commit="4a0cb7a4a737864312f8d6e3f89823a741539bfc",
        cache_path="jaxon_description",
    ),
    "jvrc_description": Repository(
        url="https://github.com/jrl-umi3218/jvrc_description.git",
        commit="9ff8efbc7043459a8f0892662bd030d8020fb682",
        cache_path="jvrc_description",
    ),
    "jvrc_mj_description": Repository(
        url="https://github.com/isri-aist/jvrc_mj_description.git",
        commit="0f0ce7daefdd66c54e0909a6bf2c22154844f5f3",
        cache_path="jvrc_mj_description",
    ),
    "LEAP_Hand_Sim": Repository(
        url="https://github.com/leap-hand/LEAP_Hand_Sim.git",
        commit="150bc3d4b61fd6619193ba5a8ef209f3609ced89",
        cache_path="LEAP_Hand_Sim",
    ),
    "kinova_mj_description": Repository(
        url="https://github.com/mathieu-celerier/kinova_mj_description.git",
        commit="cee556b0a438e6904634a90826d4e8d2e005cd1f",
        cache_path="kinova_mj_description",
    ),
    "mini_cheetah_urdf": Repository(
        url="https://github.com/Derek-TH-Wang/mini_cheetah_urdf.git",
        commit="1988bceb26e81f28594a16e7d5e6abe5cbb27ace",
        cache_path="mini_cheetah_urdf",
    ),
    "mujoco_menagerie": Repository(
        url="https://github.com/deepmind/mujoco_menagerie.git",
        commit="e44555e5a6478f5119dfd5ca9c4a8270ae1b9933",
        cache_path="mujoco_menagerie",
    ),
    "nao_robot": Repository(
        url="https://github.com/ros-naoqi/nao_robot.git",
        commit="67476469a1371b00b17538eb6ea336367ece7d44",
        cache_path="nao_robot",
    ),
    "nasa-urdf-robots": Repository(
        url="https://github.com/gkjohnson/nasa-urdf-robots.git",
        commit="54cdeb1dbfb529b79ae3185a53e24fce26e1b74b",
        cache_path="nasa-urdf-robots",
    ),
    "onshape-to-robot-examples": Repository(
        url="https://github.com/Rhoban/onshape-to-robot-examples.git",
        commit="911abb069c781e4c717c10643b975f55f7a64fe8",
        cache_path="onshape-to-robot-examples",
    ),
    "pepper_description": Repository(
        url="https://github.com/jrl-umi3218/pepper_description.git",
        commit="cd9715bb5df7ad57445d953db7b1924255305944",
        cache_path="pepper_description",
    ),
    "Piper_ros": Repository(
        url="https://github.com/agilexrobotics/Piper_ros.git",
        commit="f2ec6a67e1f404bcb478529e89861ccdf43fa298",
        cache_path="Piper_ros",
    ),
    "poppy_ergo_jr_description": Repository(
        url="https://github.com/poppy-project/poppy_ergo_jr_description.git",
        commit="7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b",
        cache_path="poppy_ergo_jr_description",
    ),
    "poppy_torso_description": Repository(
        url="https://github.com/poppy-project/poppy_torso_description.git",
        commit="6beeec3d76fb72b7548cce7c73aad722f8884522",
        cache_path="poppy_torso_description",
    ),
    "reachy_description": Repository(
        url="https://github.com/aubrune/reachy_description.git",
        commit="release-1.0.0",
        cache_path="reachy_description",
    ),
    "rhea_description": Repository(
        url="https://github.com/G-Levine/rhea_description.git",
        commit="1dc0f1abcf51b5d8a8f7ff8a548399ff0df1414f",
        cache_path="rhea_description",
    ),
    "robot-assets": Repository(
        url="https://github.com/ankurhanda/robot-assets.git",
        commit="12f1a3c89c9975194551afaed0dfae1e09fdb27c",
        cache_path="robot-assets",
    ),
    "roboschool": Repository(
        url="https://github.com/openai/roboschool.git",
        commit="1.0.49",
        cache_path="roboschool",
    ),
    "robotiq_arg85_description": Repository(
        url="https://github.com/a-price/robotiq_arg85_description.git",
        commit="a65190bdbb0666609fe7e8c3bb17341e09e81625",
        cache_path="robotiq_arg85_description",
    ),
    "romeo_robot": Repository(
        url="https://github.com/ros-aldebaran/romeo_robot.git",
        commit="0.1.5",
        cache_path="romeo_robot",
    ),
    "ros2_kortex": Repository(
        url="https://github.com/Kinovarobotics/ros2_kortex.git",
        commit="fb633aacf9c1c85d61a96c1099b5afa3d533e5a3",
        cache_path="ros2_kortex",
    ),
    "rtmros_nextage": Repository(
        url="https://github.com/tork-a/rtmros_nextage.git",
        commit="ac270fb969fa54abeb6863f9b388a9e20c1f14e0",
        cache_path="rtmros_nextage",
    ),
    "sigmaban_urdf": Repository(
        url="https://github.com/Rhoban/sigmaban_urdf.git",
        commit="d5d023fd35800d00d7647000bce8602617a4960d",
        cache_path="sigmaban_urdf",
    ),
    "simple_humanoid_description": Repository(
        url="https://github.com/laas/simple_humanoid_description.git",
        commit="4e859aed7df3c29954c9cca2a1ecb94069f7cfce",
        cache_path="simple_humanoid_description",
    ),
    "SO-ARM100": Repository(
        url="https://github.com/TheRobotStudio/SO-ARM100.git",
        commit="8967344301571dfa22660c73a901ad00acd6ee91",
        cache_path="SO-ARM100",
    ),
    "spryped": Repository(
        url="https://github.com/bbokser/spryped.git",
        commit="f360a6b78667a4d97c86cad465ef8f4c9512462b",
        cache_path="spryped",
    ),
    "stretch_description": Repository(
        url="https://github.com/robot-descriptions/stretch_description.git",
        commit="4b838429fe4c5d9f2937efe698444bd68968f376",
        cache_path="stretch_description",
    ),
    "talos-data": Repository(
        url="https://github.com/stack-of-tasks/talos-data.git",
        commit="77169405d6a48a5d3f3f75eb014209f375ff23b6",  # v2.0.0
        cache_path="talos_data",  # match package name
    ),
    "unitree_mujoco": Repository(
        url="https://github.com/unitreerobotics/unitree_mujoco.git",
        commit="f3300ff1bf0ab9efbea0162717353480d9b05d73",
        cache_path="unitree_mujoco",
    ),
    "unitree_ros": Repository(
        url="https://github.com/unitreerobotics/unitree_ros.git",
        commit="8bca7f8907bf6feceac80809ae67e4b379ab9cfc",
        cache_path="unitree_ros",
    ),
    "upkie_description": Repository(
        url="https://github.com/upkie/upkie_description.git",
        commit="19a91ce69cab6742c613cab104986e3f8a18d6a5",
        cache_path="upkie_description",
    ),
    "skydio_x2_description": Repository(
        url="https://github.com/lvjonok/skydio_x2_description.git",
        commit="9a6a057a055babaf47119fac42c361fffc189128",
        cache_path="skydio_x2_description",
    ),
}
