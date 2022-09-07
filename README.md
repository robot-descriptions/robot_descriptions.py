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

- [MuJoCo](examples/load_in_mujoco.py)
- [Pinocchio](examples/load_in_pinocchio.py)
- [PyBullet](examples/load_in_pybullet.py)

Visualize a robot description:

- [MeshCat](examples/show_in_meshcat.py)
- [MuJoCo](examples/show_in_mujoco.py)
- [PyBullet](examples/show_in_pybullet.py)
- [yourdfpy](examples/show_in_yourdfpy.py)

## Command line tool

The command line tool can be used to visualize any of the robot descriptions below. For example:

```console
robot_descriptions show solo_description
```

## Descriptions

Available robot descriptions ([gallery](https://github.com/robot-descriptions/awesome-robot-descriptions#gallery)) currently include:

| Name                  | Format | Maker              | Submodule                     |
|-----------------------|--------|--------------------| ------------------------------|
| A1                    | URDF   | UNITREE Robotics   | `a1_description`              |
| Aliengo               | MJCF   | UNITREE Robotics   | `aliengo_mj_description`      |
| Aliengo               | URDF   | UNITREE Robotics   | `aliengo_description`         |
| Allegro Hand          | URDF   | Wonik Robotics     | `allegro_hand_description`    |
| ANYmal B              | URDF   | ANYbotics          | `anymal_b_description`        |
| ANYmal C              | URDF   | ANYbotics          | `anymal_c_description`        |
| Atlas                 | URDF   | Boston Dynamics    | `atlas_description`           |
| Baxter                | URDF   | Rethink Robotics   | `baxter_description`          |
| Bolt                  | URDF   | ODRI               | `bolt_description`            |
| Cassie                | MJCF   | Agility Robotics   | `cassie_mj_description`       |
| Cassie                | URDF   | Agility Robotics   | `cassie_description`          |
| Crazyflie 2.0         | URDF   | Bitcraze           | `cf2_description`             |
| Double Pendulum       | URDF   | N/A                | `double_pendulum_description` |
| e.DO                  | URDF   | Comau              | `edo_description`             |
| FingerEdu             | URDF   | N/A                | `finger_edu_description`      |
| Gen2                  | URDF   | Kinova             | `gen2_description`            |
| HyQ                   | URDF   | IIT                | `hyq_description`             |
| iCub                  | URDF   | IIT                | `icub_description`            |
| iiwa                  | URDF   | KUKA               | `iiwa_description`            |
| JVRC-1                | MJCF   | AIST               | `jvrc_mj_description`         |
| JVRC-1                | URDF   | AIST               | `jvrc_description`            |
| Laikago               | URDF   | UNITREE Robotics   | `laikago_description`         |
| Mini Cheetah          | URDF   | MIT                | `mini_cheetah_description`    |
| Minitaur              | URDF   | Ghost Robotics     | `minitaur_description`        |
| Panda                 | URDF   | Franka Emika       | `panda_description`           |
| PR2                   | URDF   | Willow Garage      | `pr2_description`             |
| Reachy                | URDF   | Pollen Robotics    | `reachy_description`          |
| Romeo                 | URDF   | Aldebaran Robotics | `romeo_description`           |
| Simple Humanoid       | URDF   | N/A                | `simple_humanoid_description` |
| Solo                  | URDF   | ODRI               | `solo_description`            |
| TALOS                 | URDF   | PAL Robotics       | `talos_description`           |
| TIAGo                 | URDF   | PAL Robotics       | `tiago_description`           |
| Upkie                 | URDF   | Tast's Robots      | `upkie_description`           |
| UR10                  | URDF   | Universal Robots   | `ur10_description`            |
| UR3                   | URDF   | Universal Robots   | `ur3_description`             |
| UR5                   | URDF   | Universal Robots   | `ur5_description`             |

New robot descriptions are welcome! Check out the [guidelines](CONTRIBUTING.md) then open a PR.

## Thanks

Thanks to the maintainers of all the git repositories that made these robot descriptions available.
