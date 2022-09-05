# Robot descriptions in Python

[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)

Import open source robot descriptions as Python modules. The wrapper automatically downloads and cache files at first import. Most [Awesome Robot Descriptions](https://github.com/stephane-caron/awesome-robot-descriptions) are available.

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
        Path to the main URDF file of the robot description.
    </dd>
    <dt>
        <code>MESHES_PATH</code>
    </dt>
    <dd>
        Path to the "meshes" folder of the robot description, if applicable.
    </dd>
    <dt>
        <code>PATH</code>
    </dt>
    <dd>
        Path to the local robot description directory.
    </dd>
</dl>

Some robot descriptions include additional fields. For instance, the ``iiwa_description`` exports ``URDF_PATH_POLYTOPE_COLLISION`` with more detailed collision meshes.

## Command line tool

The command line tool can be used to display any of the robot descriptions below. For example:

```console
robot_descriptions show solo_description
```

## Descriptions

Available robot descriptions ([gallery](https://github.com/stephane-caron/awesome-robot-descriptions#gallery)) currently include:

| Name                  | Maker              | Submodule                     |
|-----------------------|--------------------| ------------------------------|
| A1                    | UNITREE Robotics   | `a1_description`              |
| Aliengo               | UNITREE Robotics   | `aliengo_description`         |
| Allegro Hand          | Wonik Robotics     | `allegro_hand_description`    |
| ANYmal B              | ANYbotics          | `anymal_b_description`        |
| ANYmal C              | ANYbotics          | `anymal_c_description`        |
| Atlas                 | Boston Dynamics    | `atlas_description`           |
| Baxter                | Rethink Robotics   | `baxter_description`          |
| Bolt                  | ODRI               | `bolt_description`            |
| Cassie                | Agility Robotics   | `cassie_description`          |
| Crazyflie 2.0         | Bitcraze           | `cf2_description`             |
| Double Pendulum       | N/A                | `double_pendulum_description` |
| e.DO                  | Comau              | `edo_description`             |
| FingerEdu             | N/A                | `finger_edu_description`      |
| Gen2                  | Kinova             | `gen2_description`            |
| HyQ                   | IIT                | `hyq_description`             |
| iCub                  | IIT                | `icub_description`            |
| iiwa                  | KUKA               | `iiwa_description`            |
| JVRC-1                | AIST               | `jvrc_description`            |
| Laikago               | UNITREE Robotics   | `laikago_description`         |
| Mini Cheetah          | MIT                | `mini_cheetah_description`    |
| Minitaur              | Ghost Robotics     | `minitaur_description`        |
| Panda                 | Franka Emika       | `panda_description`           |
| PR2                   | Willow Garage      | `pr2_description`             |
| Reachy                | Pollen Robotics    | `reachy_description`          |
| Romeo                 | Aldebaran Robotics | `romeo_description`           |
| Simple Humanoid       | N/A                | `simple_humanoid_description` |
| Solo                  | ODRI               | `solo_description`            |
| TALOS                 | PAL Robotics       | `talos_description`           |
| TIAGo                 | PAL Robotics       | `tiago_description`           |
| Upkie                 | Tast's Robots      | `upkie_description`           |
| UR10                  | Universal Robots   | `ur10_description`            |
| UR3                   | Universal Robots   | `ur3_description`             |
| UR5                   | Universal Robots   | `ur5_description`             |

New robot descriptions are welcome! Check out the [guidelines](CONTRIBUTING.md) then open a PR.
