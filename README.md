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

[![Conda Version](https://img.shields.io/conda/vn/conda-forge/robot_descriptions.svg)](https://anaconda.org/conda-forge/robot_descriptions)

```console
conda install -c conda-forge robot_descriptions
```

### From PyPI

[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)

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

### Show a description

You can display a robot description directly from the command line:

```console
python -m robot_descriptions show_in_meshcat go2_description
```

A `robot_descriptions` alias for `python -m robot_descriptions` is also available.

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

### Arms

| Name                          | Robot                 | Maker                    | Format     | License      |
|-------------------------------|-----------------------|--------------------------|------------|--------------|
| `edo_description`             | e.DO                  | Comau                    | URDF       | BSD-3-Clause |
| `fanuc_m710ic_description`    | M-710iC               | Fanuc                    | URDF       | BSD-3-Clause |
| `fr3_mj_description`          | FR3                   | Franka Robotics          | MJCF       | Apache-2.0   |
| `gen2_description`            | Gen2                  | Kinova                   | URDF       | BSD-3-Clause |
| `gen3_description`            | Gen3                  | Kinova                   | URDF       | MIT          |
| `gen3_mj_description`         | Gen3                  | Kinova                   | MJCF       | BSD-2-Clause |
| `iiwa14_description`          | iiwa 14               | KUKA                     | URDF       | BSD-3-Clause |
| `iiwa14_mj_description`       | iiwa 14               | KUKA                     | MJCF       | BSD-3-Clause |
| `iiwa7_description`           | iiwa 7                | KUKA                     | URDF       | MIT          |
| `panda_description`           | Panda                 | Franka Robotics          | URDF       | Apache-2.0   |
| `panda_mj_description`        | Panda                 | Franka Robotics          | MJCF       | Apache-2.0   |
| `piper_description`           | PiPER                 | AgileX                   | URDF       | MIT          |
| `piper_mj_description`        | PiPER                 | AgileX                   | MJCF       | MIT          |
| `poppy_ergo_jr_description`   | Poppy Ergo Jr         | Poppy Project            | URDF       | GPL-3.0      |
| `sawyer_mj_description`       | Sawyer                | Rethink Robotics         | MJCF       | Apache-2.0   |
| `so_arm100`                   | SO-ARM100             | The Robot Studio         | URDF       | Apache-2.0   |
| `so_arm100_mj_description`    | SO-ARM100             | The Robot Studio         | MJCF       | Apache-2.0   |
| `ur10_description`            | UR10                  | Universal Robots         | URDF       | Apache-2.0   |
| `ur10e_mj_description`        | UR10e                 | Universal Robots         | MJCF       | BSD-3-Clause |
| `ur3_description`             | UR3                   | Universal Robots         | URDF       | Apache-2.0   |
| `ur5_description`             | UR5                   | Universal Robots         | URDF       | Apache-2.0   |
| `ur5e_mj_description`         | UR5e                  | Universal Robots         | MJCF       | BSD-3-Clause |
| `viper_mj_description`        | ViperX                | Trossen Robotics         | MJCF       | BSD-3-Clause |
| `widow_mj_description`        | WidowX                | Trossen Robotics         | MJCF       | BSD-3-Clause |
| `xarm7_mj_description`        | xArm7                 | UFACTORY                 | MJCF       | BSD-3-Clause |
| `z1_description`              | Z1                    | UNITREE Robotics         | URDF       | BSD-3-Clause |

### Bipeds

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `bolt_description`            | Bolt                  | ODRI                     | 6   | URDF       |
| `cassie_description`          | Cassie                | Agility Robotics         | 16  | URDF       |
| `cassie_mj_description`       | Cassie                | Agility Robotics         | 16  | MJCF       |
| `rhea_description`            | Rhea                  | Gabrael Levine           | 7   | URDF       |
| `spryped_description`         | Spryped               | Benjamin Bokser          | 8   | URDF       |
| `upkie_description`           | Upkie                 | Tast's Robots            | 6   | URDF       |

### Dual arms

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `aloha_mj_description`        | Aloha 2               | Trossen Robotics         | 14  | MJCF       |
| `baxter_description`          | Baxter                | Rethink Robotics         | 15  | URDF       |
| `nextage_description`         | NEXTAGE               | Kawada Robotics          | 15  | URDF       |
| `poppy_torso_description`     | Poppy Torso           | Poppy Project            | 13  | URDF       |
| `yumi_description`            | YuMi                  | ABB                      | 16  | URDF       |

### Drones

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `cf2_description`             | Crazyflie 2.0         | Bitcraze                 | 0   | URDF       |
| `cf2_mj_description`          | Crazyflie 2.0         | Bitcraze                 | 6   | MJCF       |
| `skydio_x2_description`       | Skydio X2             | Skydio                   | 6   | URDF       |
| `skydio_x2_mj_description`    | Skydio X2             | Skydio                   | 6   | MJCF       |

### Educational

| Name                          | Robot                 | DOF | Format     |
|-------------------------------|-----------------------|-----|------------|
| `double_pendulum_description` | Double Pendulum       | 2   | URDF       |
| `finger_edu_description`      | FingerEdu             | 3   | URDF       |
| `simple_humanoid_description` | Simple Humanoid       | 29  | URDF       |
| `trifinger_edu_description`   | TriFingerEdu          | 9   | URDF       |

### End effectors

| Name                             | Robot                 | Maker                      | DOF | Format     |
|----------------------------------|-----------------------|----------------------------|-----|------------|
| `allegro_hand_description`       | Allegro Hand          | Wonik Robotics             | 16  | URDF       |
| `allegro_hand_mj_description`    | Allegro Hand          | Wonik Robotics             | 16  | MJCF       |
| `barrett_hand_description`       | BarrettHand           | Barrett Technology         | 8   | URDF       |
| `leap_hand_v1`                   | LEAP Hand v1          | Carnegie Mellon University | 16  | URDF       |
| `leap_hand_mj_description`       | LEAP Hand             | Carnegie Mellon University | 16  | MJCF       |
| `robotiq_2f85_description`       | Robotiq 2F-85         | Robotiq                    | 1   | URDF       |
| `robotiq_2f85_mj_description`    | Robotiq 2F-85         | Robotiq                    | 1   | MJCF       |
| `robotiq_2f85_v4_mj_description` | Robotiq 2F-85         | Robotiq                    | 1   | MJCF       |
| `shadow_dexee_mj_description`    | Shadow DEX-EE         | The Shadow Robot Company   | 12  | MJCF       |
| `shadow_hand_mj_description`     | Shadow Hand           | The Shadow Robot Company   | 24  | MJCF       |

### Humanoids

| Name                            | Robot                 | Maker                    | DOF | Format     |
|---------------------------------|-----------------------|--------------------------|-----|------------|
| `atlas_drc_description`         | Atlas DRC (v3)        | Boston Dynamics          | 30  | URDF       |
| `atlas_v4_description`          | Atlas v4              | Boston Dynamics          | 30  | URDF       |
| `berkeley_humanoid_description` | Berkeley Humanoid     | Hybrid Robotics          | 12  | URDF       |
| `draco3_description`            | Draco3                | Apptronik                | 25  | URDF       |
| `ergocub_description`           | ergoCub               | IIT                      | 57  | URDF       |
| `g1_description`                | G1                    | UNITREE Robotics         | 37  | URDF       |
| `g1_mj_description`             | G1                    | UNITREE Robotics         | 37  | MJCF       |
| `h1_description`                | H1                    | UNITREE Robotics         | 25  | URDF       |
| `h1_mj_description`             | H1                    | UNITREE Robotics         | 25  | MJCF       |
| `icub_description`              | iCub                  | IIT                      | 32  | URDF       |
| `jaxon_description`             | JAXON                 | JSK                      | 38  | URDF       |
| `jvrc_description`              | JVRC-1                | AIST                     | 34  | URDF       |
| `jvrc_mj_description`           | JVRC-1                | AIST                     | 34  | MJCF       |
| `op3_mj_description`            | OP3                   | ROBOTIS                  | 20  | MJCF       |
| `r2_description`                | Robonaut 2            | NASA JSC Robotics        | 56  | URDF       |
| `romeo_description`             | Romeo                 | Aldebaran Robotics       | 37  | URDF       |
| `sigmaban_description`          | SigmaBan              | Rhoban                   | 20  | URDF       |
| `talos_description`             | TALOS                 | PAL Robotics             | 32  | URDF       |
| `talos_mj_description`          | TALOS                 | PAL Robotics             | 32  | MJCF       |
| `valkyrie_description`          | Valkyrie              | NASA JSC Robotics        | 59  | URDF       |

### Mobile manipulators

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `eve_r3_description`          | Eve R3                | Halodi                   | 23  | URDF       |
| `fetch_description`           | Fetch                 | Fetch Robotics           | 14  | URDF       |
| `ginger_description`          | Ginger                | Paaila Technology        | 49  | URDF       |
| `pepper_description`          | Pepper                | SoftBank Robotics        | 17  | URDF       |
| `pr2_description`             | PR2                   | Willow Garage            | 32  | URDF       |
| `reachy_description`          | Reachy                | Pollen Robotics          | 21  | URDF       |
| `stretch_description`         | Stretch RE1           | Hello Robot              | 14  | URDF       |
| `sretch_mj_description`       | Stretch 2             | Hello Robot              | 14  | MJCF       |
| `sretch_3_mj_description`     | Stretch 3             | Hello Robot              | 14  | MJCF       |
| `tiago_description`           | TIAGo                 | PAL Robotics             | 48  | URDF       |

### Quadrupeds

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `a1_description`              | A1                    | UNITREE Robotics         | 12  | URDF       |
| `a1_mj_description`           | A1                    | UNITREE Robotics         | 12  | MJCF       |
| `aliengo_description`         | Aliengo               | UNITREE Robotics         | 12  | URDF       |
| `aliengo_mj_description`      | Aliengo               | UNITREE Robotics         | 12  | MJCF       |
| `anymal_b_description`        | ANYmal B              | ANYbotics                | 12  | URDF       |
| `anymal_b_mj_description`     | ANYmal B              | ANYbotics                | 12  | MJCF       |
| `anymal_c_description`        | ANYmal C              | ANYbotics                | 12  | URDF       |
| `anymal_c_mj_description`     | ANYmal C              | ANYbotics                | 12  | MJCF       |
| `anymal_d_description`        | ANYmal D              | ANYbotics                | 12  | URDF       |
| `b1_description`              | B1                    | UNITREE Robotics         | 12  | URDF       |
| `b2_description`              | B2                    | UNITREE Robotics         | 12  | URDF       |
| `spot_mj_description`         | Spot                  | Boston Dynamics          | 12  | MJCF       |
| `go1_description`             | Go1                   | UNITREE Robotics         | 12  | URDF       |
| `go1_mj_description`          | Go1                   | UNITREE Robotics         | 12  | MJCF       |
| `go2_description`             | Go2                   | UNITREE Robotics         | 12  | URDF       |
| `go2_mj_description`          | Go2                   | UNITREE Robotics         | 12  | MJCF       |
| `hyq_description`             | HyQ                   | IIT                      | 12  | URDF       |
| `laikago_description`         | Laikago               | UNITREE Robotics         | 12  | MJCF, URDF |
| `mini_cheetah_description`    | Mini Cheetah          | MIT                      | 12  | URDF       |
| `minitaur_description`        | Minitaur              | Ghost Robotics           | 16  | URDF       |
| `solo_description`            | Solo                  | ODRI                     | 12  | URDF       |

### Wheeled

| Name                          | Robot                 | Maker                    | Format     | License |
|-------------------------------|-----------------------|--------------------------|------------|---------|
| `rsk_description`             | RSK Omnidirectional   | Robot Soccer Kit         | URDF       | MIT     |

## Contributing

New robot descriptions are welcome! Check out the [guidelines](https://github.com/robot-descriptions/robot_descriptions.py/tree/main/CONTRIBUTING.md) then open a PR.

## Thanks

Thanks to the maintainers of all the git repositories that made these robot descriptions available.

## Citation

If you use this project in your works, please cite as follows:

```bibtex
@software{robot_descriptions_py,
  title = {{robot_descriptions.py: Robot descriptions in Python}},
  author = {Caron, Stéphane and Romualdi, Giulio and Kozlov, Lev and Ordoñez Apraez, Daniel Felipe and Tadashi Kussaba, Hugo and Bang, Seung Hyeon and Zakka, Kevin and Schramm, Fabian and Uru\c{c}, Jafar},
  license = {Apache-2.0},
  url = {https://github.com/robot-descriptions/robot_descriptions.py},
  version = {1.15.0},
  year = {2025}
}
```

## See also

- [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions): curated list of robot descriptions in URDF, Xacro or MJCF formats.
- [drake\_models](https://github.com/RobotLocomotion/models): collection of URDF and SDF descriptions curated for the Drake framework.
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/): collection of MJCF robot descriptions curated for the MuJoCo physics engine.
- [robot\_descriptions.cpp](https://github.com/mayataka/robot_descriptions.cpp): package to use ``robot_descriptions.py`` in C++.
