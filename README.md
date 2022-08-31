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

| Robot name            | Submodule                     |
| --------------------- | ----------------------------- |
| Atlas                 | `atlas_description`           |
| Allegro Hand          | `allegro_hand_description`    |
| Crazyflie 2.0         | `cf2_description`             |
| Double Pendulum       | `double_pendulum_description` |
| e.DO                  | `edo_description`             |
| FingerEdu v1          | `finger_edu_description`      |
| Kinova Gen2           | `kinova_description`          |
| KUKA iiwa 14          | `iiwa_description`            |
| Panda                 | `panda_description`           |
| Universal Robots UR3  | `ur3_description`             |
| Universal Robots UR5  | `ur5_description`             |
| Universal Robots UR10 | `ur10_description`            |
| Upkie                 | `upkie_description`           |

New models are welcome! Check out the [guidelines](CONTRIBUTING.md) before opening a PR.
