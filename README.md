# Robot descriptions as Python modules

[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)

Import open source robot models as Python modules. The wrapper automatically downloads and cache model files at first import. URDF descriptions from [Awesome Robot Models](https://github.com/stephane-caron/awesome-robot-models) are available.

## Installation

```console
pip install robot_descriptions
```

## Usage

Import the robot description you are interested in directly as a submodule of ``robot_descriptions``:

```python
from robot_descriptions import my_robot_description
```

The import will automatically download and cache the robot model, if you don't have it already. The submodule then exports the following paths:

<dl>
    <dt>
        <code>MESHES_PATH</code>
    </dt>
    <dd>
        Path to the "meshes" folder in the robot description.
    </dd>
    <dt>
        <code>PATH</code>
    </dt>
    <dd>
        Path to the root directory of the robot description.
    </dd>
    <dt>
        <code>URDF_PATH</code>
    </dt>
    <dd>
        Path to the URDF file of the robot model.
    </dd>
</dl>

Some robot descriptions include additional fields. For instance, the ``ur3_description`` exports ``GRIPPER_URDF_PATH`` for the optional end effector.

## Models

Available robot models ([gallery](https://github.com/stephane-caron/awesome-robot-models#gallery)) currently include:

| Name                  | Maker            | Submodule                     |
|-----------------------|------------------| ------------------------------|
| Allegro Hand          | Wonik Robotics   | `allegro_hand_description`    |
| Atlas                 | Boston Dynamics  | `atlas_description`           |
| Baxter                | Rethink Robotics | `baxter_description`          |
| Bolt                  | ODRI             | `bolt_description`            |
| Cassie                | Agility Robotics | `cassie_description`          |
| Crazyflie 2.0         | Bitcraze         | `cf2_description`             |
| Double Pendulum       | N/A              | `double_pendulum_description` |
| e.DO                  | Comau            | `edo_description`             |
| FingerEdu             | N/A              | `finger_edu_description`      |
| Gen2                  | Kinova           | `gen2_description`            |
| iCub                  | IIT              | `icub_description`            |
| iiwa                  | KUKA             | `iiwa_description`            |
| JVRC-1                | AIST             | `jvrc_description`            |
| Panda                 | Franka Emika     | `panda_description`           |
| PR2                   | Willow Garage    | `pr2_description`             |
| Upkie                 | Tast's Robots    | `upkie_description`           |
| UR3                   | Universal Robots | `ur3_description`             |
| UR5                   | Universal Robots | `ur5_description`             |
| UR10                  | Universal Robots | `ur10_description`            |

New models are welcome! Check out the [guidelines](CONTRIBUTING.md) before opening a PR.
