# Robot descriptions in Python

[![Build](https://img.shields.io/github/workflow/status/stephane-caron/robot_descriptions.py/CI)](https://github.com/stephane-caron/robot_descriptions.py/actions)
[![Coverage](https://coveralls.io/repos/github/stephane-caron/robot_descriptions.py/badge.svg?branch=master)](https://coveralls.io/github/stephane-caron/robot_descriptions.py?branch=master)
[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)
![Status](https://img.shields.io/pypi/status/robot_descriptions)

Import open source robot descriptions as Python modules. The wrapper automatically downloads and cache files at first import. Most [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions) are available.

## Installation

```console
pip install robot_descriptions
```

## Usage

Import the robot description you are interested in directly as a submodule of ``robot_descriptions``:

```python
from robot_descriptions import my_robot_description
```

The import will automatically download the robot description if you don't have it already, and cache it to a local directory. The submodule then provides the following paths:

<dl>
    <dt>
        <code>URDF_PATH</code>
    </dt>
    <dd>
        Path to the main URDF file of the robot description, if applicable.
    </dd>
    <dt>
        <code>MJCF_PATH</code>
    </dt>
    <dd>
        Path to the main MJCF file of the robot description, if applicable.
    </dd>
    <dt>
        <code>MESHES_PATH</code>
    </dt>
    <dd>
        Path to the "meshes" folder of the robot description, if applicable.
    </dd>
    <dt>
        <code>PACKAGE_PATH</code>
    </dt>
    <dd>
        Path to the root of the robot description.
    </dd>
    <dt>
        <code>REPOSITORY_PATH</code>
    </dt>
    <dd>
        Path to the working directory of the git repository hosting the robot description.
    </dd>
</dl>

Some robot descriptions include additional fields. For instance, the ``iiwa_description`` exports ``URDF_PATH_POLYTOPE_COLLISION`` with more detailed collision meshes.

## Examples

Load a robot description:

- [MuJoCo](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/load_in_mujoco.py)
- [Pinocchio](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/load_in_pinocchio.py)
- [PyBullet](https://github.com/stephane-caron/robot_descriptions.py/tree/master/examples/load_in_pybullet.py)

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

| Name                  | Maker              | Submodule                     | Format     |
|-----------------------|--------------------| ------------------------------|------------|
| A1                    | UNITREE Robotics   | `a1_description`              | MJCF, URDF |
| Aliengo               | UNITREE Robotics   | `aliengo_description`         | MJCF, URDF |
| Allegro Hand          | Wonik Robotics     | `allegro_hand_description`    | URDF       |
| ANYmal B              | ANYbotics          | `anymal_b_description`        | URDF       |
| ANYmal C              | ANYbotics          | `anymal_c_description`        | URDF       |
| Atlas                 | Boston Dynamics    | `atlas_description`           | URDF       |
| Baxter                | Rethink Robotics   | `baxter_description`          | URDF       |
| Bolt                  | ODRI               | `bolt_description`            | URDF       |
| Cassie                | Agility Robotics   | `cassie_description`          | URDF       |
| Cassie                | Agility Robotics   | `cassie_mj_description`       | MJCF       |
| Crazyflie 2.0         | Bitcraze           | `cf2_description`             | URDF       |
| Double Pendulum       | N/A                | `double_pendulum_description` | URDF       |
| e.DO                  | Comau              | `edo_description`             | URDF       |
| FingerEdu             | N/A                | `finger_edu_description`      | URDF       |
| Gen2                  | Kinova             | `gen2_description`            | URDF       |
| Go1                   | UNITREE Robotics   | `go1_description`             | MJCF, URDF |
| HyQ                   | IIT                | `hyq_description`             | URDF       |
| iCub                  | IIT                | `icub_description`            | URDF       |
| iiwa                  | KUKA               | `iiwa_description`            | URDF       |
| JVRC-1                | AIST               | `jvrc_description`            | URDF       |
| JVRC-1                | AIST               | `jvrc_mj_description`         | MJCF       |
| Laikago               | UNITREE Robotics   | `laikago_description`         | MJCF, URDF |
| Mini Cheetah          | MIT                | `mini_cheetah_description`    | URDF       |
| Minitaur              | Ghost Robotics     | `minitaur_description`        | URDF       |
| Panda                 | Franka Emika       | `panda_description`           | URDF       |
| PR2                   | Willow Garage      | `pr2_description`             | URDF       |
| Reachy                | Pollen Robotics    | `reachy_description`          | URDF       |
| Romeo                 | Aldebaran Robotics | `romeo_description`           | URDF       |
| Simple Humanoid       | N/A                | `simple_humanoid_description` | URDF       |
| Solo                  | ODRI               | `solo_description`            | URDF       |
| TALOS                 | PAL Robotics       | `talos_description`           | URDF       |
| TIAGo                 | PAL Robotics       | `tiago_description`           | URDF       |
| Upkie                 | Tast's Robots      | `upkie_description`           | URDF       |
| UR10                  | Universal Robots   | `ur10_description`            | URDF       |
| UR3                   | Universal Robots   | `ur3_description`             | URDF       |
| UR5                   | Universal Robots   | `ur5_description`             | URDF       |

New robot descriptions are welcome! Check out the [guidelines](https://github.com/stephane-caron/robot_descriptions.py/tree/master/CONTRIBUTING.md) then open a PR.

## Thanks

Thanks to the maintainers of all the git repositories that made these robot descriptions available.
