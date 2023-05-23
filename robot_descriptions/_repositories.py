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
    "baxter_common": Repository(
        url="https://github.com/RethinkRobotics/baxter_common.git",
        commit="v1.2.0",
        cache_path="baxter_common",
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
        commit="v1.7.0",
        cache_path="drake",
    ),
    "eDO_description": Repository(
        url="https://github.com/Comau/eDO_description.git",
        commit="17b3f92f834746106d6a4befaab8eeab3ac248e6",
        cache_path="edo_sim",  # match package name
    ),
    "ergocub-software": Repository(
        url="https://github.com/icub-tech-iit/ergocub-software.git",
        commit="ac3f223dc2f183dea3f819369da0d58b59f1b2d3",
        cache_path="ergocub-software",
    ),
    "example-robot-data": Repository(
        url="https://github.com/Gepetto/example-robot-data.git",
        commit="9ba565ca1491efa92ebac38cdd499e5b1c256bf1",  # v4.0.3
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
        commit="v1.0.0",
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
        url="https://github.com/robot-descriptions/jvrc_description.git",
        commit="b788e1a7c874b3447fc027e175f216c6192e9e96",
        cache_path="jvrc_description",
    ),
    "jvrc_mj_description": Repository(
        url="https://github.com/isri-aist/jvrc_mj_description.git",
        commit="0f0ce7daefdd66c54e0909a6bf2c22154844f5f3",
        cache_path="jvrc_mj_description",
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
        commit="e1d346bfdbbb7e89997a12ce787dd1316e20e704",
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
    "pepper_description": Repository(
        url="https://github.com/jrl-umi3218/pepper_description.git",
        commit="cd9715bb5df7ad57445d953db7b1924255305944",
        cache_path="pepper_description",
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
        commit="0e488ee4708155a71b2a92d05305a9186b543593",
        cache_path="simple_humanoid_description",
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
        commit="v2.0.0",
        cache_path="talos_data",  # match package name
    ),
    "unitree_mujoco": Repository(
        url="https://github.com/unitreerobotics/unitree_mujoco.git",
        commit="f3300ff1bf0ab9efbea0162717353480d9b05d73",
        cache_path="unitree_mujoco",
    ),
    "unitree_ros": Repository(
        url="https://github.com/unitreerobotics/unitree_ros.git",
        commit="059bd619c14c5d06346bcdec57b4191dccee2b86",
        cache_path="unitree_ros",
    ),
    "upkie_description": Repository(
        url="https://github.com/tasts-robots/upkie_description.git",
        commit="3bc5251b7255641d275de8c6b949e13bf1a608ea",
        cache_path="upkie_description",
    ),
}
