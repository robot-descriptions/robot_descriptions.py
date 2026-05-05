# Robot descriptions in Python

[![Build](https://img.shields.io/github/actions/workflow/status/robot-descriptions/robot_descriptions.py/test.yml?branch=main)](https://github.com/robot-descriptions/robot_descriptions.py/actions)
[![Coverage](https://coveralls.io/repos/github/robot-descriptions/robot_descriptions.py/badge.svg?branch=main)](https://coveralls.io/github/robot-descriptions/robot_descriptions.py?branch=main)
[![Conda Version](https://img.shields.io/conda/vn/conda-forge/robot_descriptions.svg?color=blue)](https://anaconda.org/conda-forge/robot_descriptions)
[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions?color=blue)](https://pypi.org/project/robot_descriptions/)
[![PyPI downloads](https://img.shields.io/pypi/dm/robot_descriptions?color=blue)](https://pypistats.org/packages/robot_descriptions)
[![Contributing](https://img.shields.io/badge/PRs-welcome-green.svg)](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/CONTRIBUTING.md)

Import open source robot descriptions as Python modules.

Importing a description for the first time automatically downloads and caches files for future imports. Most [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions) are available. All of them load successfully in respectively MuJoCo (MJCF) or Pinocchio, iDynTree, PyBullet and yourdfpy (URDF).

## Installation

### From conda-forge

```console
conda install -c conda-forge robot_descriptions
```

### From PyPI

```console
pip install robot_descriptions
```

## Usage

The library provides `load_robot_description` functions that return an instance of a robot description directly usable in the corresponding software. For example:

```python
from robot_descriptions.loaders.pinocchio import load_robot_description

robot = load_robot_description("upkie_description")
```

Loaders are implemented for the following robotics software:

| Software                                                     | Loader                                   |
|--------------------------------------------------------------|------------------------------------------|
| [iDynTree](https://github.com/robotology/idyntree)           | `robot_descriptions.loaders.idyntree`    |
| [MuJoCo](https://github.com/deepmind/mujoco)                 | `robot_descriptions.loaders.mujoco`      |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio)     | `robot_descriptions.loaders.pinocchio`   |
| [PyBullet](https://pybullet.org/)                            | `robot_descriptions.loaders.pybullet`    |
| [RoboMeshCat](https://github.com/petrikvladimir/RoboMeshCat) | `robot_descriptions.loaders.robomeshcat` |
| [yourdfpy](https://github.com/clemense/yourdfpy/)            | `robot_descriptions.loaders.yourdfpy`    |

Loading will automatically download the robot description if needed, and cache it to a local directory.

### Command line

You can use [uv](https://docs.astral.sh/uv/) to manipulate robot descriptions directly from the command line:

```console
uvx robot_descriptions pull iiwa14_description          # download the robot description to cache
uvx robot_descriptions show_in_meshcat go2_description  # display the robot description
```

Try `uvx robot_descriptions -h` to see the list of available commands. Alternatively, once the package is installed from PyPI or conda-forge, you can call the `robot_descriptions` alias directly from the command line, or replace `uvx` by `uv run`, or `python -m robot_descriptions` (old school).

### Import as submodule

You can also import a robot description directly as a submodule of `robot_descriptions`:

```python
from robot_descriptions import my_robot_description
```

The import will automatically download the robot description if you don't have it already, and cache it to a local directory. The submodule then provides the following paths:

<dl>
    <dt>
        <code>URDF_PATH</code> / <code>MJCF_PATH</code>
    </dt>
    <dd>
        Path to the main URDF/MJCF file of the robot description.
    </dd>
    <dt>
        <code>PACKAGE_PATH</code>
    </dt>
    <dd>
        Path to the root of the robot description package.
    </dd>
    <dt>
        <code>REPOSITORY_PATH</code>
    </dt>
    <dd>
        Path to the working directory of the git repository hosting the robot description.
    </dd>
</dl>

Some robot descriptions include additional fields. For instance, the `iiwa14_description` exports `URDF_PATH_POLYTOPE_COLLISION` with more detailed collision meshes.

Descriptions can also define `XACRO_PATH` (and optional `XACRO_ARGS`) instead
of a static `URDF_PATH`. In that case, `robot_descriptions` renders the Xacro
to a cached URDF transparently when loading the description.

## Examples

Loading a robot description:

- [iDynTree](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/load_in_idyntree.py)
- [MuJoCo](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/load_in_mujoco.py)
- [Pinocchio](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/load_in_pinocchio.py)
- [PyBullet](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/load_in_pybullet.py)
- [RoboMeshCat](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/load_in_robomeshcat.py)
- [yourdfpy](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/load_in_yourdfpy.py)

Visualizing a robot description:

- [MeshCat](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/show_in_meshcat.py)
- [MuJoCo](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/show_in_mujoco.py)
- [PyBullet](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/show_in_pybullet.py)
- [RoboMeshCat](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/show_in_robomeshcat.py)
- [yourdfpy](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/examples/show_in_yourdfpy.py)

## Descriptions

Available robot descriptions ([gallery](https://github.com/robot-descriptions/awesome-robot-descriptions#gallery)) are listed in the following categories:

- [Arms](#arms)
- [Bipeds](#bipeds)
- [Dual arms](#dual-arms)
- [Drones](#drones)
- [Educational](#educational)
- [End effectors](#end-effectors)
- [Humanoids](#humanoids)
- [Mobile manipulators](#mobile-manipulators)
- [Quadrupeds](#quadrupeds)
- [Wheeled](#wheeled)

The DOF column denotes the number of actuated degrees of freedom.
Descriptions may appear in more than one category when tags overlap.

<!-- BEGIN GENERATED DESCRIPTION TABLES -->

### Arms

| Name                                | Robot              | Maker            | Format | License                                                                                                                                                                                                |
|-------------------------------------|--------------------|------------------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `arx_l5_mj_description`             | L5                 | ARX              | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/arx_l5/LICENSE)                                                                              |
| `edo_description`                   | e.DO               | Comau            | URDF   | [BSD-3-Clause](https://github.com/ianathompson/eDO_description/blob/17b3f92f834746106d6a4befaab8eeab3ac248e6/LICENSE)                                                                                  |
| `fanuc_m710ic_description`          | M-710iC            | Fanuc            | URDF   | [BSD-3-Clause](https://github.com/robot-descriptions/fanuc_m710ic_description/blob/d12af44559cd7e46f7afd513237f159f82f8402e/LICENSE)                                                                   |
| `fer_description`                   | FER                | Franka Robotics  | URDF   | [Apache-2.0](https://github.com/frankarobotics/franka_description/blob/72baf5bf4e88eaec27f0eb61be1b20a001abf2ab/LICENSE)                                                                               |
| `fr3_description`                   | FR3                | Franka Robotics  | URDF   | [Apache-2.0](https://github.com/frankarobotics/franka_description/blob/72baf5bf4e88eaec27f0eb61be1b20a001abf2ab/LICENSE)                                                                               |
| `fr3_mj_description`                | FR3                | Franka Robotics  | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/franka_fr3/LICENSE)                                                                            |
| `fr3_v2_1_description`              | FR3 v2.1           | Franka Robotics  | URDF   | [Apache-2.0](https://github.com/frankarobotics/franka_description/blob/72baf5bf4e88eaec27f0eb61be1b20a001abf2ab/LICENSE)                                                                               |
| `fr3_v2_description`                | FR3 v2             | Franka Robotics  | URDF   | [Apache-2.0](https://github.com/frankarobotics/franka_description/blob/72baf5bf4e88eaec27f0eb61be1b20a001abf2ab/LICENSE)                                                                               |
| `fr3_v2_mj_description`             | FR3 v2             | Franka Robotics  | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/franka_fr3_v2/LICENSE)                                                                         |
| `gen2_description`                  | Gen2               | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)                                                                                    |
| `gen3_description`                  | Gen3               | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/ros2_kortex/blob/8bf203423911446de28a2248ec87380b7eea2f90/LICENSE)                                                                                    |
| `gen3_lite_description`             | Gen3 Lite          | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/ros2_kortex/blob/8bf203423911446de28a2248ec87380b7eea2f90/LICENSE)                                                                                    |
| `gen3_mj_description`               | Gen3               | Kinova           | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/kinova_gen3/LICENSE)                                                                         |
| `iiwa14_description`                | iiwa 14            | KUKA             | URDF   | [BSD-3-Clause](https://github.com/RobotLocomotion/drake/blob/7abea0556ede980a5077fe1a8cfbae59b57c7c27/manipulation/models/iiwa_description/LICENSE.TXT)                                                |
| `iiwa14_mj_description`             | iiwa 14            | KUKA             | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/kuka_iiwa_14/LICENSE)                                                                        |
| `iiwa7_description`                 | iiwa 7             | KUKA             | URDF   | [MIT](https://github.com/facebookresearch/differentiable-robot-model/blob/d7bd1b3b8ef1d6dabe9b68474a622185c510e112/LICENSE)                                                                            |
| `j2n4s300_description`              | Jaco2 j2n4s300     | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/kinova-ros/blob/924781d3dfe241b2b94b3c72a804b80d3658cf02/LICENSE)                                                                                     |
| `j2n6s200_description`              | Jaco2 j2n6s200     | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/kinova-ros/blob/924781d3dfe241b2b94b3c72a804b80d3658cf02/LICENSE)                                                                                     |
| `j2n6s300_description`              | Jaco2 j2n6s300     | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/kinova-ros/blob/924781d3dfe241b2b94b3c72a804b80d3658cf02/LICENSE)                                                                                     |
| `j2n7s300_description`              | Jaco2 j2n7s300     | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/kinova-ros/blob/924781d3dfe241b2b94b3c72a804b80d3658cf02/LICENSE)                                                                                     |
| `j2s6s200_description`              | Jaco2 j2s6s200     | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/kinova-ros/blob/924781d3dfe241b2b94b3c72a804b80d3658cf02/LICENSE)                                                                                     |
| `j2s6s300_description`              | Jaco2 j2s6s300     | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/kinova-ros/blob/924781d3dfe241b2b94b3c72a804b80d3658cf02/LICENSE)                                                                                     |
| `j2s7s300_description`              | Jaco2 j2s7s300     | Kinova           | URDF   | [BSD-3-Clause](https://github.com/Kinovarobotics/kinova-ros/blob/924781d3dfe241b2b94b3c72a804b80d3658cf02/LICENSE)                                                                                     |
| `low_cost_robot_arm_mj_description` | Low-cost robot arm | Alexander Koch   | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/low_cost_robot_arm/LICENSE)                                                                    |
| `omx_f_description`                 | OMX-F              | ROBOTIS          | URDF   | [Apache-2.0](https://github.com/ROBOTIS-GIT/open_manipulator/blob/bc555a9c41ebd7493dc945ddabc43fc649681b62/LICENSE)                                                                                    |
| `omx_l_description`                 | OMX-L              | ROBOTIS          | URDF   | [Apache-2.0](https://github.com/ROBOTIS-GIT/open_manipulator/blob/bc555a9c41ebd7493dc945ddabc43fc649681b62/LICENSE)                                                                                    |
| `omy_3m_description`                | OMY-3M             | ROBOTIS          | URDF   | [Apache-2.0](https://github.com/ROBOTIS-GIT/open_manipulator/blob/bc555a9c41ebd7493dc945ddabc43fc649681b62/LICENSE)                                                                                    |
| `omy_f3m_description`               | OMY-F3M            | ROBOTIS          | URDF   | [Apache-2.0](https://github.com/ROBOTIS-GIT/open_manipulator/blob/bc555a9c41ebd7493dc945ddabc43fc649681b62/LICENSE)                                                                                    |
| `omy_l100_description`              | OMY-L100           | ROBOTIS          | URDF   | [Apache-2.0](https://github.com/ROBOTIS-GIT/open_manipulator/blob/bc555a9c41ebd7493dc945ddabc43fc649681b62/LICENSE)                                                                                    |
| `open_manipulator_x_description`    | OpenMANIPULATOR-X  | ROBOTIS          | URDF   | [Apache-2.0](https://github.com/ROBOTIS-GIT/open_manipulator/blob/bc555a9c41ebd7493dc945ddabc43fc649681b62/LICENSE)                                                                                    |
| `panda_description`                 | Panda              | Franka Robotics  | URDF   | [Apache-2.0](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/robots/panda_description/LICENSE)                                                             |
| `panda_mj_description`              | Panda              | Franka Robotics  | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/franka_emika_panda/LICENSE)                                                                    |
| `piper_description`                 | PiPER              | AgileX           | URDF   | [MIT](https://github.com/agilexrobotics/Piper_ros/blob/f2ec6a67e1f404bcb478529e89861ccdf43fa298/LICENSE)                                                                                               |
| `piper_mj_description`              | PiPER              | AgileX           | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/agilex_piper/LICENSE)                                                                                 |
| `poppy_ergo_jr_description`         | Poppy Ergo Jr      | Poppy Project    | URDF   | [GPL-3.0](https://github.com/poppy-project/poppy_ergo_jr_description/blob/7eb32bd385afa11dea5e6a6b6a4a86a0243aaa2b/package.xml)                                                                        |
| `rizon4_description`                | Rizon4             | Flexiv Robotics  | URDF   | [Apache-2.0](https://github.com/flexivrobotics/flexiv_description/blob/edb01274caf7eaf2ba722bbfdf2f23d717fee38e/LICENSE)                                                                               |
| `rizon4_mj_description`             | Rizon4             | Flexiv Robotics  | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/LICENSE)                                                                                       |
| `sawyer_mj_description`             | Sawyer             | Rethink Robotics | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/rethink_robotics_sawyer/LICENSE)                                                               |
| `so_arm100_description`             | SO-ARM100          | The Robot Studio | URDF   | [Apache-2.0](https://github.com/TheRobotStudio/SO-ARM100/blob/63eede5a636e548eb8f2854e558bd343c21db9f7/LICENSE)                                                                                        |
| `so_arm100_mj_description`          | SO-ARM100          | The Robot Studio | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/trs_so_arm100/LICENSE)                                                                         |
| `so_arm101_description`             | SO-ARM101          | The Robot Studio | URDF   | [Apache-2.0](https://github.com/TheRobotStudio/SO-ARM100/blob/63eede5a636e548eb8f2854e558bd343c21db9f7/LICENSE)                                                                                        |
| `so_arm101_mj_description`          | SO-ARM101          | The Robot Studio | MJCF   | [Apache-2.0](https://github.com/TheRobotStudio/SO-ARM100/blob/63eede5a636e548eb8f2854e558bd343c21db9f7/LICENSE)                                                                                        |
| `ur10_description`                  | UR10               | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)                                                                                    |
| `ur10_official_description`         | UR10 (official)    | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur10e_description`                 | UR10e              | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur10e_mj_description`              | UR10e              | Universal Robots | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/universal_robots_ur10e/LICENSE)                                                              |
| `ur12e_description`                 | UR12e              | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur15_description`                  | UR15               | Universal Robots | URDF   | [Universal Robots Terms for Graphical Documentation ✖️](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/meshes/ur15/LICENSE.txt)    |
| `ur16e_description`                 | UR16e              | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur18_description`                  | UR18               | Universal Robots | URDF   | [Universal Robots Terms for Graphical Documentation ✖️](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/meshes/ur18/LICENSE.txt)    |
| `ur20_description`                  | UR20               | Universal Robots | URDF   | [Universal Robots Terms for Graphical Documentation ✖️](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/meshes/ur20/LICENSE.txt)    |
| `ur30_description`                  | UR30               | Universal Robots | URDF   | [Universal Robots Terms for Graphical Documentation ✖️](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/meshes/ur30/LICENSE.txt)    |
| `ur3_description`                   | UR3                | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)                                                                                    |
| `ur3_official_description`          | UR3 (official)     | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur3e_description`                  | UR3e               | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur5_description`                   | UR5                | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)                                                                                    |
| `ur5_official_description`          | UR5 (official)     | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur5e_description`                  | UR5e               | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur5e_mj_description`               | UR5e               | Universal Robots | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/universal_robots_ur5e/LICENSE)                                                               |
| `ur7e_description`                  | UR7e               | Universal Robots | URDF   | [BSD-3-Clause](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/LICENSE)                                                             |
| `ur8long_description`               | UR8 Long           | Universal Robots | URDF   | [Universal Robots Terms for Graphical Documentation ✖️](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/22f055da2fa7e2158254426107d1f257fd56aebb/meshes/ur8long/LICENSE.txt) |
| `viper_mj_description`              | ViperX             | Trossen Robotics | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/trossen_vx300s/LICENSE)                                                                      |
| `widow_mj_description`              | WidowX             | Trossen Robotics | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/trossen_wx250s/LICENSE)                                                                      |
| `xarm6_description`                 | xArm6              | UFACTORY         | URDF   | [BSD-3-Clause](https://github.com/xArm-Developer/xarm_ros2/blob/5bb832f72ca665f1236a9d8ed1c3a82f308db489/LICENSE)                                                                                      |
| `xarm7_description`                 | xArm7              | UFACTORY         | URDF   | [BSD-3-Clause](https://github.com/xArm-Developer/xarm_ros2/blob/5bb832f72ca665f1236a9d8ed1c3a82f308db489/LICENSE)                                                                                      |
| `xarm7_mj_description`              | xArm7              | UFACTORY         | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/ufactory_xarm7/LICENSE)                                                                      |
| `yam_description`                   | YAM                | I2RT Robotics    | URDF   | [MIT](https://github.com/robot-descriptions/i2rt/blob/7809b5b17227162d265f922e2e10598c0e214322/LICENSE)                                                                                                |
| `yam_mj_description`                | YAM                | I2RT Robotics    | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/i2rt_yam/LICENSE)                                                                                     |
| `z1_description`                    | Z1                 | UNITREE Robotics | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                                                                                   |
| `z1_mj_description`                 | Z1                 | UNITREE Robotics | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/unitree_z1/LICENSE)                                                                          |

### Bipeds

| Name                    | Robot   | Maker            | DOF | Format | License                                                                                                                  |
|-------------------------|---------|------------------|-----|--------|--------------------------------------------------------------------------------------------------------------------------|
| `bolt_description`      | Bolt    | ODRI             | 6   | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)      |
| `cassie_description`    | Cassie  | Agility Robotics | 16  | URDF   | [MIT](https://github.com/robot-descriptions/cassie_description/blob/81a2d8bbd77201cc974afb127adda4e2857a6dbf/LICENSE)    |
| `cassie_mj_description` | Cassie  | Agility Robotics | 16  | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/agility_cassie/LICENSE) |
| `rhea_description`      | Rhea    | Gabrael Levine   | 7   | URDF   | [Apache-2.0](https://github.com/G-Levine/rhea_description/blob/1dc0f1abcf51b5d8a8f7ff8a548399ff0df1414f/LICENSE)         |
| `spryped_description`   | Spryped | Benjamin Bokser  | 8   | URDF   | [BSD](https://github.com/bbokser/spryped/blob/f360a6b78667a4d97c86cad465ef8f4c9512462b/spryped_urdf_rev06/package.xml)   |
| `upkie_description`     | Upkie   | Tast's Robots    | 6   | URDF   | [Apache-2.0](https://github.com/upkie/upkie_description/blob/19a91ce69cab6742c613cab104986e3f8a18d6a5/LICENSE)           |

### Dual arms

| Name                        | Robot       | Maker            | DOF | Format | License                                                                                                                       |
|-----------------------------|-------------|------------------|-----|--------|-------------------------------------------------------------------------------------------------------------------------------|
| `aloha_mj_description`      | Aloha 2     | Trossen Robotics | 14  | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/aloha/LICENSE)      |
| `baxter_description`        | Baxter      | Rethink Robotics | 15  | URDF   | [BSD-3-Clause](https://github.com/RethinkRobotics/baxter_common/blob/6c4b0f375fe4e356a3b12df26ef7c0d5e58df86e/LICENSE)        |
| `nextage_description`       | NEXTAGE     | Kawada Robotics  | 15  | URDF   | [BSD](https://github.com/tork-a/rtmros_nextage/blob/ac270fb969fa54abeb6863f9b388a9e20c1f14e0/nextage_description/package.xml) |
| `openarm_v1_mj_description` | OpenArm     | Enactic          | 16  | MJCF   | [Apache-2.0](https://github.com/enactic/openarm_mujoco/blob/cd30dd4c0a97832d1c063bf759514ed18fbe04a5/LICENSE)                 |
| `poppy_torso_description`   | Poppy Torso | Poppy Project    | 13  | URDF   | [GPL-3.0](https://github.com/poppy-project/poppy_torso_description/blob/6beeec3d76fb72b7548cce7c73aad722f8884522/package.xml) |
| `pr2_description`           | PR2         | Willow Garage    |     | URDF   | [BSD](https://github.com/ankurhanda/robot-assets/blob/12f1a3c89c9975194551afaed0dfae1e09fdb27c/README.md)                     |
| `yumi_description`          | YuMi        | ABB              | 16  | URDF   | [BSD](https://github.com/ankurhanda/robot-assets/blob/12f1a3c89c9975194551afaed0dfae1e09fdb27c/README.md)                     |

### Drones

| Name                       | Robot         | Maker    | DOF | Format | License                                                                                                                        |
|----------------------------|---------------|----------|-----|--------|--------------------------------------------------------------------------------------------------------------------------------|
| `cf2_description`          | Crazyflie 2.0 | Bitcraze | 0   | URDF   | [MIT](https://github.com/utiasDSL/gym-pybullet-drones/blob/2c2470c2043eb4cf474d9af2681c9fa10f2d0e2f/LICENSE)                   |
| `cf2_mj_description`       | Crazyflie 2.0 | Bitcraze | 6   | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/bitcraze_crazyflie_2/LICENSE) |
| `skydio_x2_description`    | Skydio X2     | Skydio   | 6   | URDF   | [Apache-2.0](https://github.com/lvjonok/skydio_x2_description/blob/9a6a057a055babaf47119fac42c361fffc189128/LICENSE)           |
| `skydio_x2_mj_description` | Skydio X2     | Skydio   | 6   | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/skydio_x2/LICENSE)     |

### Educational

| Name                             | Robot           | DOF | Format | License                                                                                                                     |
|----------------------------------|-----------------|-----|--------|-----------------------------------------------------------------------------------------------------------------------------|
| `double_pendulum_description`    | Double Pendulum | 2   | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)         |
| `dynamixel_2r_mj_description`    | Dynamixel 2R    | 2   | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/dynamixel_2r/LICENSE)      |
| `finger_edu_description`         | FingerEdu       | 3   | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)         |
| `mujoco_humanoid_mj_description` | MuJoCo Humanoid | 27  | MJCF   | [Apache-2.0](https://github.com/google-deepmind/mujoco/blob/ad0dc0de5e10a075a2c65be629e9a8d557d383a6/LICENSE)               |
| `simple_humanoid_description`    | Simple Humanoid | 29  | URDF   | [BSD-2-Clause](https://github.com/laas/simple_humanoid_description/blob/4e859aed7df3c29954c9cca2a1ecb94069f7cfce/LICENSE)   |
| `trifinger_edu_description`      | TriFingerEdu    | 9   | URDF   | [MIT](https://github.com/facebookresearch/differentiable-robot-model/blob/d7bd1b3b8ef1d6dabe9b68474a622185c510e112/LICENSE) |

### End effectors

| Name                                     | Robot                      | Maker                      | Format | License                                                                                                                                                     |
|------------------------------------------|----------------------------|----------------------------|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `ability_hand_description`               | Ability Hand               | PSYONIC, Inc.              | URDF   | [MIT](https://github.com/psyonicinc/ability-hand-api/blob/89407424edfc22faceaedcd7c3ea2b7947cbbb2c/LICENSE)                                                 |
| `ability_hand_mj_description`            | Ability Hand               | PSYONIC, Inc.              | MJCF   | [MIT](https://github.com/psyonicinc/ability-hand-api/blob/89407424edfc22faceaedcd7c3ea2b7947cbbb2c/LICENSE)                                                 |
| `aero_hand_open_description`             | Aero Hand Open             | TetherIA                   | URDF   | [CC-BY-SA-4.0 ✖️](https://github.com/TetherIA/aero-hand-open/blob/ad7d4fc4078ceedfeefbf27aa5f058b622e18521/ros2/src/aero_hand_open_description/package.xml) |
| `aero_hand_open_mj_description`          | Aero Hand Open             | TetherIA                   | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/tetheria_aero_hand_open/LICENSE)                    |
| `allegro_hand_description`               | Allegro Hand               | Wonik Robotics             | URDF   | [BSD](https://github.com/RobotLocomotion/drake/blob/7abea0556ede980a5077fe1a8cfbae59b57c7c27/manipulation/models/allegro_hand_description/LICENSE.TXT)      |
| `allegro_hand_mj_description`            | Allegro Hand               | Wonik Robotics             | MJCF   | [BSD-2-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/wonik_allegro/LICENSE)                            |
| `barrett_hand_description`               | BarrettHand                | Barrett Technology         | URDF   | [BSD](https://github.com/jhu-lcsr-attic/bhand_model/blob/937f4186d6458bd682a7dae825fb6f4efe56ec69/manifest.xml)                                             |
| `leap_hand_mj_description`               | LEAP Hand                  | Carnegie Mellon University | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/leap_hand/LICENSE)                                         |
| `leap_hand_v1_description`               | LEAP Hand v1               | Carnegie Mellon University | URDF   | [MIT](https://github.com/leap-hand/LEAP_Hand_Sim/blob/150bc3d4b61fd6619193ba5a8ef209f3609ced89/LICENSE.txt)                                                 |
| `robotiq_2f85_description`               | Robotiq 2F-85              | Robotiq                    | URDF   | [BSD](https://github.com/a-price/robotiq_arg85_description/blob/a65190bdbb0666609fe7e8c3bb17341e09e81625/package.xml)                                       |
| `robotiq_2f85_mj_description`            | Robotiq 2F-85              | Robotiq                    | MJCF   | [BSD-2-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/robotiq_2f85/LICENSE)                             |
| `robotiq_2f85_v4_description`            | Robotiq 2F-85 v4           | Robotiq                    | URDF   | [BSD-2-Clause](https://github.com/nickswalker/robotiq-2f-85/blob/319f7b94587c97f09e732a7b2ed7babbe0250b9e/LICENSE)                                          |
| `robotiq_2f85_v4_mj_description`         | Robotiq 2F-85              | Robotiq                    | MJCF   | [BSD-2-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/robotiq_2f85_v4/LICENSE)                          |
| `shadow_dexee_mj_description`            | Shadow DEX-EE              | The Shadow Robot Company   | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/shadow_dexee/LICENSE)                               |
| `shadow_hand_mj_description`             | Shadow Hand                | The Shadow Robot Company   | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/shadow_hand/LICENSE)                                |
| `so_arm101_parallel_gripper_description` | SO-ARM101 Parallel Gripper | Robonine                   | URDF   | [GPL-3.0](https://github.com/roboninecom/SO-ARM100-101-Parallel-Gripper/blob/ed44013818de822369d0f0c9461ae0c20a3e6bb8/LICENSE)                              |

### Humanoids

| Name                             | Robot             | Maker               | DOF | Format | License                                                                                                                                   |
|----------------------------------|-------------------|---------------------|-----|--------|-------------------------------------------------------------------------------------------------------------------------------------------|
| `adam_lite_mj_description`       | Adam Lite         | PNDBotics           |     | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/pndbotics_adam_lite/LICENSE)             |
| `apollo_mj_description`          | Apollo            | Apptronik           |     | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/apptronik_apollo/LICENSE)         |
| `atlas_drc_description`          | Atlas DRC (v3)    | Boston Dynamics     |     | URDF   | [BSD-3-Clause](https://github.com/RobotLocomotion/drake/blob/7abea0556ede980a5077fe1a8cfbae59b57c7c27/LICENSE.TXT)                        |
| `atlas_v4_description`           | Atlas v4          | Boston Dynamics     |     | URDF   | [MIT](https://github.com/openai/roboschool/blob/1.0.49/LICENSE.md)                                                                        |
| `berkeley_humanoid_description`  | Berkeley Humanoid | Hybrid Robotics     |     | URDF   | [BSD-3-Clause](https://github.com/HybridRobotics/berkeley_humanoid_description/blob/d0d13d3f81d795480e25ed1910eaf83d5f0a1d0b/package.xml) |
| `booster_t1_description`         | Booster T1        | Booster Robotics    |     | URDF   | [Apache-2.0](https://github.com/BoosterRobotics/booster_gym/blob/687a33d08b08875fe45dc8d91b54db83766df8b9/LICENSE)                        |
| `booster_t1_mj_description`      | Booster T1        | Booster Robotics    |     | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/booster_t1/LICENSE)               |
| `draco3_description`             | Draco3            | Apptronik           |     | URDF   | [BSD-2-Clause](https://github.com/shbang91/draco3_description/blob/5afd19733d7b3e9f1135ba93e0aad90ed1a24cc7/LICENSE)                      |
| `elf2_description`               | Elf2              | BXI Robotics        |     | URDF   | [Apache-2.0](https://github.com/bxirobotics/robot_models/blob/eabe24ce937f8e633077a163b883e92e8996c36e/LICENSE)                           |
| `elf2_mj_description`            | Elf2              | BXI Robotics        |     | MJCF   | [Apache-2.0](https://github.com/bxirobotics/robot_models/blob/eabe24ce937f8e633077a163b883e92e8996c36e/LICENSE)                           |
| `ergocub_description`            | ergoCub           | IIT                 |     | URDF   | [BSD-3-Clause](https://github.com/icub-tech-iit/ergocub-software/blob/v0.7.7/LICENSE)                                                     |
| `g1_description`                 | G1                | UNITREE Robotics    |     | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                      |
| `g1_mj_description`              | G1                | UNITREE Robotics    |     | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/unitree_g1/LICENSE)             |
| `gr1_description`                | GR-1              | Fourier             |     | URDF   | [GPL-3.0](https://github.com/FFTAI/Wiki-GRx-Models/blob/351245ac8fa4bf6f4b0c41556e1e6976a438bcef/LICENSE)                                 |
| `h1_2_description`               | H1_2              | UNITREE Robotics    |     | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                      |
| `h1_2_mj_description`            | H1_2              | UNITREE Robotics    |     | MJCF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                      |
| `h1_description`                 | H1                | UNITREE Robotics    |     | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                      |
| `h1_mj_description`              | H1                | UNITREE Robotics    |     | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/unitree_h1/LICENSE)             |
| `icub_description`               | iCub              | IIT                 |     | URDF   | [CC-BY-SA-4.0 ✖️](https://github.com/robotology/icub-models/blob/v1.25.0/iCub/package.xml)                                                |
| `jaxon_description`              | JAXON             | JSK                 |     | URDF   | [CC-BY-SA-4.0 ✖️](https://github.com/robot-descriptions/jaxon_description/blob/4a0cb7a4a737864312f8d6e3f89823a741539bfc/LICENSE)          |
| `jvrc_description`               | JVRC-1            | AIST                |     | URDF   | [BSD-2-Clause](https://github.com/jrl-umi3218/jvrc_description/blob/9ff8efbc7043459a8f0892662bd030d8020fb682/LICENSE)                     |
| `jvrc_mj_description`            | JVRC-1            | AIST                |     | MJCF   | [BSD-2-Clause](https://github.com/isri-aist/jvrc_mj_description/blob/0f0ce7daefdd66c54e0909a6bf2c22154844f5f3/LICENSE)                    |
| `mujoco_humanoid_mj_description` | MuJoCo Humanoid   |                     | 27  | MJCF   | [Apache-2.0](https://github.com/google-deepmind/mujoco/blob/ad0dc0de5e10a075a2c65be629e9a8d557d383a6/LICENSE)                             |
| `n1_description`                 | N1                | Fourier             |     | URDF   | [Apache-2.0](https://github.com/FFTAI/Wiki-GRx-Models/blob/f8e683f00d1d99deb882deb9dfce6030095b466a/LICENSE)                              |
| `n1_mj_description`              | N1                | Fourier             |     | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/fourier_n1/LICENSE)               |
| `op3_mj_description`             | OP3               | ROBOTIS             |     | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/robotis_op3/LICENSE)              |
| `r2_description`                 | Robonaut 2        | NASA JSC Robotics   |     | URDF   | [NASA-1.3](https://github.com/gkjohnson/nasa-urdf-robots/blob/54cdeb1dbfb529b79ae3185a53e24fce26e1b74b/README.md)                         |
| `romeo_description`              | Romeo             | Aldebaran Robotics  |     | URDF   | [BSD](https://github.com/ros-aldebaran/romeo_robot/blob/0.1.5/romeo_description/package.xml)                                              |
| `sigmaban_description`           | SigmaBan          | Rhoban              |     | URDF   | [MIT](https://github.com/Rhoban/sigmaban_urdf/blob/d5d023fd35800d00d7647000bce8602617a4960d/LICENSE)                                      |
| `simple_humanoid_description`    | Simple Humanoid   |                     | 29  | URDF   | [BSD-2-Clause](https://github.com/laas/simple_humanoid_description/blob/4e859aed7df3c29954c9cca2a1ecb94069f7cfce/LICENSE)                 |
| `talos_description`              | TALOS             | PAL Robotics        |     | URDF   | [LGPL-3.0](https://github.com/stack-of-tasks/talos-data/blob/77169405d6a48a5d3f3f75eb014209f375ff23b6/LICENSE)                            |
| `talos_mj_description`           | TALOS             | PAL Robotics        |     | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/pal_talos/LICENSE)                |
| `toddlerbot_2xc_mj_description`  | ToddlerBot 2XC    | Stanford University |     | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/toddlerbot_2xc/LICENSE)                  |
| `toddlerbot_2xm_mj_description`  | ToddlerBot 2XM    | Stanford University |     | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/toddlerbot_2xm/LICENSE)                  |
| `toddlerbot_description`         | ToddlerBot        | Stanford University |     | URDF   | [MIT](https://github.com/hshi74/toddlerbot/blob/067f9dc4f50143e36334877b9395b9c5c29ee30c/LICENSE)                                         |
| `valkyrie_description`           | Valkyrie          | NASA JSC Robotics   |     | URDF   | [NASA-1.3](https://github.com/gkjohnson/nasa-urdf-robots/blob/54cdeb1dbfb529b79ae3185a53e24fce26e1b74b/README.md)                         |

### Mobile manipulators

| Name                       | Robot       | Maker             | Format | License                                                                                                                                              |
|----------------------------|-------------|-------------------|--------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| `bambot_description`       | BamBot      | Tim Qian          | URDF   | [Apache-2.0](https://github.com/timqian/bambot/blob/04d902653794f9f72eeabb09ec90a9af8e397c5b/LICENSE)                                                |
| `eve_r3_description`       | Eve R3      | Halodi            | URDF   | [Apache-2.0](https://github.com/Halodi/halodi-robot-models/blob/ba9e7c8cdbd63e20fc6526dbbea1b91c102fb820/eve_r3_description/package.xml)             |
| `fetch_description`        | Fetch       | Fetch Robotics    | URDF   | [MIT](https://github.com/openai/roboschool/blob/1.0.49/LICENSE.md)                                                                                   |
| `ginger_description`       | Ginger      | Paaila Technology | URDF   | [BSD](https://github.com/Rayckey/GingerURDF/blob/6a1307cd0ee2b77c82f8839cdce3a2e2eed2bd8f/package.xml)                                               |
| `pepper_description`       | Pepper      | SoftBank Robotics | URDF   | [BSD-2-Clause](https://github.com/jrl-umi3218/pepper_description/blob/cd9715bb5df7ad57445d953db7b1924255305944/LICENSE)                              |
| `pr2_description`          | PR2         | Willow Garage     | URDF   | [BSD](https://github.com/ankurhanda/robot-assets/blob/12f1a3c89c9975194551afaed0dfae1e09fdb27c/README.md)                                            |
| `rby1_description`         | RBY1        | Rainbow Robotics  | URDF   | [MIT](https://github.com/uynitsuj/rby1_description/blob/e4c07203aa0a0d1b6b3b39da105cb00a77e2bc72/LICENSE)                                            |
| `reachy_description`       | Reachy      | Pollen Robotics   | URDF   | [Apache-2.0](https://github.com/aubrune/reachy_description/blob/release-1.0.0/LICENSE)                                                               |
| `stretch_3_mj_description` | Stretch 3   | Hello Robot       | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/hello_robot_stretch_3/LICENSE)               |
| `stretch_description`      | Stretch RE1 | Hello Robot       | URDF   | [CC-BY-NC-SA-4.0 ✖️](https://github.com/robot-descriptions/stretch_description/blob/4b838429fe4c5d9f2937efe698444bd68968f376/LICENSE)                |
| `stretch_mj_description`   | Stretch 2   | Hello Robot       | MJCF   | [Clear BSD](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/hello_robot_stretch/LICENSE)                  |
| `stretch_se3_description`  | Stretch SE3 | Hello Robot       | URDF   | [Clear BSD](https://github.com/hello-robot/stretch_urdf/blob/1b7cbbce808c25465017ce0a53a4173fcf97b11c/LICENSE.md)                                    |
| `tiago++_mj_description`   | TIAGo++     | PAL Robotics      | MJCF   | [Apache-2.0](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/pal_tiago_dual/LICENSE)                      |
| `tiago_description`        | TIAGo       | PAL Robotics      | URDF   | [CC-BY-NC-ND-3.0 ✖️](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/robots/tiago_description/README.md) |

### Quadrupeds

| Name                       | Robot        | Maker            | DOF | Format | License                                                                                                                                    |
|----------------------------|--------------|------------------|-----|--------|--------------------------------------------------------------------------------------------------------------------------------------------|
| `a1_description`           | A1           | UNITREE Robotics | 12  | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                       |
| `a1_mj_description`        | A1           | UNITREE Robotics | 12  | MJCF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_mujoco/blob/f3300ff1bf0ab9efbea0162717353480d9b05d73/LICENSE)                    |
| `aliengo_description`      | Aliengo      | UNITREE Robotics | 12  | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                       |
| `aliengo_mj_description`   | Aliengo      | UNITREE Robotics | 12  | MJCF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_mujoco/blob/f3300ff1bf0ab9efbea0162717353480d9b05d73/LICENSE)                    |
| `anymal_b_description`     | ANYmal B     | ANYbotics        | 12  | URDF   | [BSD-3-Clause](https://github.com/ANYbotics/anymal_b_simple_description/blob/988b5df22b84761bdf08111b1c2ccc883793f456/LICENSE)             |
| `anymal_b_mj_description`  | ANYmal B     | ANYbotics        | 12  | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/anybotics_anymal_b/LICENSE)      |
| `anymal_c_description`     | ANYmal C     | ANYbotics        | 12  | URDF   | [BSD-3-Clause](https://github.com/ANYbotics/anymal_c_simple_description/blob/f160b8f7fed840c47a6febe8e2bc78b32bf43a68/LICENSE)             |
| `anymal_c_mj_description`  | ANYmal C     | ANYbotics        | 12  | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/anybotics_anymal_c/LICENSE)      |
| `anymal_d_description`     | ANYmal D     | ANYbotics        | 12  | URDF   | [BSD-3-Clause](https://github.com/ANYbotics/anymal_d_simple_description/blob/6adc14720aab583613975e5a9d6d4fa3cfcdd081/LICENSE)             |
| `b1_description`           | B1           | UNITREE Robotics | 12  | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                       |
| `b2_description`           | B2           | UNITREE Robotics | 12  | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                       |
| `go1_description`          | Go1          | UNITREE Robotics | 12  | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                       |
| `go1_mj_description`       | Go1          | UNITREE Robotics | 12  | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/unitree_go1/LICENSE)             |
| `go2_description`          | Go2          | UNITREE Robotics | 12  | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/267182b8521c8d6a631bab1fe63836873237a525/LICENSE)                       |
| `go2_mj_description`       | Go2          | UNITREE Robotics | 12  | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/unitree_go2/LICENSE)             |
| `hyq_description`          | HyQ          | IIT              | 12  | URDF   | [Apache-2.0](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/robots/hyq_description/README.md) |
| `laikago_description`      | Laikago      | UNITREE Robotics | 12  | URDF   | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_mujoco/blob/f3300ff1bf0ab9efbea0162717353480d9b05d73/LICENSE)                    |
| `mini_cheetah_description` | Mini Cheetah | MIT              | 12  | URDF   | [BSD](https://github.com/Derek-TH-Wang/mini_cheetah_urdf/blob/1988bceb26e81f28594a16e7d5e6abe5cbb27ace/package.xml)                        |
| `minitaur_description`     | Minitaur     | Ghost Robotics   | 16  | URDF   | [BSD-2-Clause](https://github.com/bulletphysics/bullet3/blob/7dee3436e747958e7088dfdcea0e4ae031ce619e/data/quadruped/license.txt)          |
| `solo_description`         | Solo         | ODRI             | 12  | URDF   | [BSD-3-Clause](https://github.com/Gepetto/example-robot-data/blob/d0d9098d752014aec3725b07766962acf06c5418/LICENSE)                        |
| `spot_mj_description`      | Spot         | Boston Dynamics  | 12  | MJCF   | [BSD-3-Clause](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/boston_dynamics_spot/LICENSE)    |
| `wl_p311d_description`     | WL P311D     | LimX Dynamics    |     | URDF   | [Apache-2.0](https://github.com/limxdynamics/robot-description/blob/a097533372a08298d45af391cbdfc2fd2dc3da6f/LICENSE)                      |
| `wl_p311e_description`     | WL P311E     | LimX Dynamics    |     | URDF   | [Apache-2.0](https://github.com/limxdynamics/robot-description/blob/a097533372a08298d45af391cbdfc2fd2dc3da6f/LICENSE)                      |

### Wheeled

| Name                   | Robot               | Maker            | DOF | Format | License                                                                                                                    |
|------------------------|---------------------|------------------|-----|--------|----------------------------------------------------------------------------------------------------------------------------|
| `rsk_description`      | RSK Omnidirectional | Robot Soccer Kit |     | URDF   | [MIT](https://github.com/Rhoban/onshape-to-robot-examples/blob/911abb069c781e4c717c10643b975f55f7a64fe8/README.md)         |
| `rsk_mj_description`   | RSK Omnidirectional | Robot Soccer Kit |     | MJCF   | [MIT](https://github.com/deepmind/mujoco_menagerie/blob/feadf76d42f8a2162426f7d226a3b539556b3bf5/robot_soccer_kit/LICENSE) |
| `upkie_description`    | Upkie               | Tast's Robots    | 6   | URDF   | [Apache-2.0](https://github.com/upkie/upkie_description/blob/19a91ce69cab6742c613cab104986e3f8a18d6a5/LICENSE)             |
| `wl_p311d_description` | WL P311D            | LimX Dynamics    |     | URDF   | [Apache-2.0](https://github.com/limxdynamics/robot-description/blob/a097533372a08298d45af391cbdfc2fd2dc3da6f/LICENSE)      |
| `wl_p311e_description` | WL P311E            | LimX Dynamics    |     | URDF   | [Apache-2.0](https://github.com/limxdynamics/robot-description/blob/a097533372a08298d45af391cbdfc2fd2dc3da6f/LICENSE)      |
<!-- END GENERATED DESCRIPTION TABLES -->

## Contributing

New robot descriptions are welcome! Check out the [guidelines](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/CONTRIBUTING.md) then open a PR.

## Thanks

Thanks to the maintainers of all the git repositories that made these robot descriptions available.

## Citation

If you use this project in your works, please cite as follows:

```bibtex
@software{robot_descriptions_py,
  title = {{robot_descriptions.py: Robot descriptions in Python}},
  author = {Caron, Stéphane and Romualdi, Giulio and Kozlov, Lev and Ordoñez Apraez, Daniel Felipe and Tadashi Kussaba, Hugo and Bang, Seung Hyeon and Zakka, Kevin and Schramm, Fabian and Uru\c{c}, Jafar and Traversaro, Silvio and Zamora, Jonathan and Castro, Sebastian and Tao, Haixuan Xavier and Yu, Justin and Jallet, Wilson and Zhang, Yutong and Walker, Nick and Bragin, Nikita},
  license = {Apache-2.0},
  url = {https://github.com/robot-descriptions/robot_descriptions.py},
  version = {2.0.0},
  year = {2026}
}
```

Don't forget to add yourself to the BibTeX above and to `CITATION.cff` if you contribute to this repository.

## See also

- [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions): curated list of robot descriptions in URDF, Xacro or MJCF formats.
- [drake\_models](https://github.com/RobotLocomotion/models): collection of URDF and SDF descriptions curated for the Drake framework.
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/): collection of MJCF robot descriptions curated for the MuJoCo physics engine.
- [robot\_descriptions.cpp](https://github.com/mayataka/robot_descriptions.cpp): package to use ``robot_descriptions.py`` in C++.
