# Robot descriptions in Python

[![Build](https://img.shields.io/github/workflow/status/stephane-caron/robot_descriptions.py/CI)](https://github.com/stephane-caron/robot_descriptions.py/actions)
[![Coverage](https://coveralls.io/repos/github/stephane-caron/robot_descriptions.py/badge.svg?branch=master)](https://coveralls.io/github/stephane-caron/robot_descriptions.py?branch=master)
[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)
![Status](https://img.shields.io/pypi/status/robot_descriptions)
[![Contributing](https://img.shields.io/badge/PRs-welcome-green.svg)](https://github.com/stephane-caron/robot_descriptions.py/tree/master/CONTRIBUTING.md)

Import open source robot descriptions as Python modules.

Importing a description for the first time automatically downloads and caches files for future imports. Most [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions) are available. All of them [load successfully](https://github.com/stephane-caron/robot_descriptions.py#loaders) in MuJoCo (MJCF), PyBullet (URDF) and yourdfpy (URDF).

## Installation

```console
pip install robot_descriptions
```

## Other languages

| ![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=c%2B%2B&logoColor=white) | [robot\_descriptions.cpp](https://github.com/mayataka/robot_descriptions.cpp) |
|--|--|

## Usage

Import the robot description you are interested in directly as a submodule of ``robot_descriptions``:

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

Some robot descriptions include additional fields. For instance, the ``iiwa_description`` exports ``URDF_PATH_POLYTOPE_COLLISION`` with more detailed collision meshes.

### Loaders

The library also provides a `load_robot_description` function for each of the following robotics software:

| Software  | Loader                                 |
|-----------|----------------------------------------|
| MuJoCo    | `robot_descriptions.loaders.mujoco`    |
| Pinocchio | `robot_descriptions.loaders.pinocchio` |
| PyBullet  | `robot_descriptions.loaders.pybullet`  |
| yourdfpy  | `robot_descriptions.loaders.yourdfpy`  |

The function loads a robot description from its name, and returns an instance directly usable in the target software. For example:

```python
from robot_descriptions.loaders.pinocchio import load_robot_description

robot = load_robot_description("upkie_description")
```

## Examples

Load a robot description:

- [MuJoCo](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/load_in_mujoco.py)
- [Pinocchio](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/load_in_pinocchio.py)
- [PyBullet](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/load_in_pybullet.py)
- [yourdfpy](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/load_in_yourdfpy.py)

Visualize a robot description:

- [MeshCat](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/show_in_meshcat.py)
- [MuJoCo](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/show_in_mujoco.py)
- [PyBullet](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/show_in_pybullet.py)
- [yourdfpy](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/show_in_yourdfpy.py)

## Command line tool

The command line tool can be used to visualize any of the robot descriptions below. For example:

```console
robot_descriptions show solo_description
```

## Descriptions

Available robot descriptions ([gallery](https://github.com/robot-descriptions/awesome-robot-descriptions#gallery)) currently include:

| Name                  | Maker                    | Format     | Submodule                     |
|-----------------------|--------------------------|------------|-------------------------------|
| A1                    | UNITREE Robotics         | MJCF       | `a1_mj_description`           |
| A1                    | UNITREE Robotics         | URDF       | `a1_description`              |
| Aliengo               | UNITREE Robotics         | MJCF, URDF | `aliengo_description`         |
| Allegro Hand          | Wonik Robotics           | URDF       | `allegro_hand_description`    |
| ANYmal B              | ANYbotics                | MJCF       | `anymal_b_mj_description`     |
| ANYmal B              | ANYbotics                | URDF       | `anymal_b_description`        |
| ANYmal C              | ANYbotics                | MJCF       | `anymal_c_mj_description`     |
| ANYmal C              | ANYbotics                | URDF       | `anymal_c_description`        |
| Atlas DRC (v3)        | Boston Dynamics          | URDF       | `atlas_drc_description`       |
| Atlas v4              | Boston Dynamics          | URDF       | `atlas_v4_description`        |
| BarrettHand           | Barrett Technology       | URDF       | `barrett_hand_description`    |
| Baxter                | Rethink Robotics         | URDF       | `baxter_description`          |
| Bolt                  | ODRI                     | URDF       | `bolt_description`            |
| Cassie                | Agility Robotics         | MJCF       | `cassie_mj_description`       |
| Cassie                | Agility Robotics         | URDF       | `cassie_description`          |
| Crazyflie 2.0         | Bitcraze                 | URDF       | `cf2_description`             |
| Double Pendulum       | N/A                      | URDF       | `double_pendulum_description` |
| e.DO                  | Comau                    | URDF       | `edo_description`             |
| Eve R3                | Halodi                   | URDF       | `eve_r3_description`          |
| Fetch                 | Fetch Robotics           | URDF       | `fetch_description`           |
| FingerEdu             | N/A                      | URDF       | `finger_edu_description`      |
| Gen2                  | Kinova                   | URDF       | `gen2_description`            |
| Gen3                  | Kinova                   | MJCF       | `gen3_mj_description`         |
| Ginger                | Paaila Technology        | URDF       | `ginger_description`          |
| Go1                   | UNITREE Robotics         | MJCF       | `go1_mj_description`          |
| Go1                   | UNITREE Robotics         | URDF       | `go1_description`             |
| HyQ                   | IIT                      | URDF       | `hyq_description`             |
| iCub                  | IIT                      | URDF       | `icub_description`            |
| iiwa                  | KUKA                     | URDF       | `iiwa_description`            |
| JAXON                 | JSK                      | URDF       | `jaxon_description`           |
| JVRC-1                | AIST                     | MJCF       | `jvrc_mj_description`         |
| JVRC-1                | AIST                     | URDF       | `jvrc_description`            |
| Laikago               | UNITREE Robotics         | MJCF, URDF | `laikago_description`         |
| Mini Cheetah          | MIT                      | URDF       | `mini_cheetah_description`    |
| Minitaur              | Ghost Robotics           | URDF       | `minitaur_description`        |
| NEXTAGE               | Kawada Robotics          | URDF       | `nextage_description`         |
| Panda                 | Franka Emika             | MJCF       | `panda_mj_description`        |
| Panda                 | Franka Emika             | URDF       | `panda_description`           |
| Pepper                | SoftBank Robotics        | URDF       | `pepper_description`          |
| Poppy Ergo Jr         | Poppy Project            | URDF       | `poppy_ergo_jr_description`   |
| Poppy Torso           | Poppy Project            | URDF       | `poppy_torso_description`     |
| PR2                   | Willow Garage            | URDF       | `pr2_description`             |
| Reachy                | Pollen Robotics          | URDF       | `reachy_description`          |
| Robonaut 2            | NASA JSC Robotics        | URDF       | `r2_description`              |
| Robotiq 2F-85         | Robotiq                  | MJCF       | `robotiq_2f85_mj_description` |
| Robotiq 2F-85         | Robotiq                  | URDF       | `robotiq_2f85_description`    |
| Romeo                 | Aldebaran Robotics       | URDF       | `romeo_description`           |
| Shadow Hand           | The Shadow Robot Company | MJCF       | `shadow_hand_mj_description`  |
| SigmaBan              | Rhoban                   | URDF       | `sigmaban_description`        |
| Simple Humanoid       | N/A                      | URDF       | `simple_humanoid_description` |
| Solo                  | ODRI                     | URDF       | `solo_description`            |
| TALOS                 | PAL Robotics             | URDF       | `talos_description`           |
| TIAGo                 | PAL Robotics             | URDF       | `tiago_description`           |
| Upkie                 | Tast's Robots            | URDF       | `upkie_description`           |
| UR10                  | Universal Robots         | URDF       | `ur10_description`            |
| UR3                   | Universal Robots         | URDF       | `ur3_description`             |
| UR5                   | Universal Robots         | URDF       | `ur5_description`             |
| UR5e                  | Universal Robots         | MJCF       | `ur5e_mj_description`         |
| Valkyrie              | NASA JSC Robotics        | URDF       | `valkyrie_description`        |
| YuMi                  | ABB                      | URDF       | `yumi_description`            |

New robot descriptions are welcome! Check out the [guidelines](https://github.com/stephane-caron/robot_descriptions.py/tree/master/CONTRIBUTING.md) then open a PR.

## Thanks

Thanks to the maintainers of all the git repositories that made these robot descriptions available.
