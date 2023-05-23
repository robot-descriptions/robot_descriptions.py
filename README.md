# Robot descriptions in Python

[![Build](https://img.shields.io/github/actions/workflow/status/robot-descriptions/robot_descriptions.py/test.yml?branch=master)](https://github.com/robot-descriptions/robot_descriptions.py/actions)
[![Coverage](https://coveralls.io/repos/github/robot-descriptions/robot_descriptions.py/badge.svg?branch=master)](https://coveralls.io/github/robot-descriptions/robot_descriptions.py?branch=master)
[![PyPI version](https://img.shields.io/pypi/v/robot_descriptions)](https://pypi.org/project/robot_descriptions/)
[![Conda Version](https://img.shields.io/conda/vn/conda-forge/robot_descriptions.svg)](https://anaconda.org/conda-forge/robot_descriptions)
[![Contributing](https://img.shields.io/badge/PRs-welcome-green.svg)](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/CONTRIBUTING.md)

Import open source robot descriptions as Python modules.

Importing a description for the first time automatically downloads and caches files for future imports. Most [Awesome Robot Descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions) are available. All of them [load successfully](https://github.com/robot-descriptions/robot_descriptions.py#loaders) in respectively MuJoCo (MJCF) or Pinocchio, iDynTree, PyBullet and yourdfpy (URDF).

## Installation

### Install from pip
```console
pip install robot_descriptions
```

### Install from conda
```console
conda install -c conda-forge robot_descriptions
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

### Import as submodule

You can also import a robot description directly as a submodule of ``robot_descriptions``:

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

## Examples

Loading a robot description:

- [iDynTree](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/load_in_idyntree.py)
- [MuJoCo](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/load_in_mujoco.py)
- [Pinocchio](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/load_in_pinocchio.py)
- [PyBullet](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/load_in_pybullet.py)
- [RoboMeshCat](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/load_in_robomeshcat.py)
- [yourdfpy](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/load_in_yourdfpy.py)

Visualizing a robot description:

- [MeshCat](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/show_in_meshcat.py)
- [MuJoCo](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/show_in_mujoco.py)
- [PyBullet](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/show_in_pybullet.py)
- [RoboMeshCat](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/show_in_robomeshcat.py)
- [yourdfpy](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/examples/show_in_yourdfpy.py)

## Command line tool

The command line tool can be used to visualize any of the robot descriptions below. For example:

```console
robot_descriptions show solo_description
```

## Descriptions

Available robot descriptions ([gallery](https://github.com/robot-descriptions/awesome-robot-descriptions#gallery)) are listed in the following categories:

* [Arms](#arms)
* [Bipeds](#bipeds)
* [Dual arms](#dual-arms)
* [Drones](#drones)
* [Educational](#educational)
* [End effectors](#end-effectors)
* [Mobile manipulators](#mobile-manipulators)
* [Humanoids](#humanoids)
* [Quadrupeds](#quadrupeds)
* [Wheeled](#wheeled)

The DOF column denotes the number of actuated degrees of freedom.

### Arms

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `edo_description`             | e.DO                  | Comau                    | 6   | URDF       |
| `fanuc_m710ic_description`    | M-710iC               | Fanuc                    | 6   | URDF       |
| `gen2_description`            | Gen2 (Jaco)           | Kinova                   | 6   | URDF       |
| `gen3_description`            | Gen3 (Jaco)           | Kinova                   | 6   | URDF       |
| `gen3_mj_description`         | Gen3 (Jaco)           | Kinova                   | 7   | MJCF       |
| `iiwa_description`            | iiwa                  | KUKA                     | 7   | URDF       |
| `panda_description`           | Panda                 | Franka Emika             | 8   | URDF       |
| `panda_mj_description`        | Panda                 | Franka Emika             | 8   | MJCF       |
| `poppy_ergo_jr_description`   | Poppy Ergo Jr         | Poppy Project            | 6   | URDF       |
| `ur10_description`            | UR10                  | Universal Robots         | 6   | URDF       |
| `ur3_description`             | UR3                   | Universal Robots         | 6   | URDF       |
| `ur5_description`             | UR5                   | Universal Robots         | 6   | URDF       |
| `ur5e_mj_description`         | UR5e                  | Universal Robots         | 6   | MJCF       |
| `z1_description`              | Z1                    | UNITREE Robotics         | 6   | URDF       |

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
| `baxter_description`          | Baxter                | Rethink Robotics         | 15  | URDF       |
| `nextage_description`         | NEXTAGE               | Kawada Robotics          | 15  | URDF       |
| `poppy_torso_description`     | Poppy Torso           | Poppy Project            | 13  | URDF       |
| `yumi_description`            | YuMi                  | ABB                      | 16  | URDF       |

### Drones

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `cf2_description`             | Crazyflie 2.0         | Bitcraze                 | 0   | URDF       |

### Educational

| Name                          | Robot                 | DOF | Format     |
|-------------------------------|-----------------------|-----|------------|
| `double_pendulum_description` | Double Pendulum       | 2   | URDF       |
| `finger_edu_description`      | FingerEdu             | 3   | URDF       |
| `simple_humanoid_description` | Simple Humanoid       | 29  | URDF       |
| `trifinger_edu_description`   | TriFingerEdu          | 9   | URDF       |

### End effectors

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `allegro_hand_description`    | Allegro Hand          | Wonik Robotics           | 16  | URDF       |
| `barrett_hand_description`    | BarrettHand           | Barrett Technology       | 8   | URDF       |
| `robotiq_2f85_description`    | Robotiq 2F-85         | Robotiq                  | 1   | URDF       |
| `robotiq_2f85_mj_description` | Robotiq 2F-85         | Robotiq                  | 1   | MJCF       |
| `shadow_hand_mj_description`  | Shadow Hand           | The Shadow Robot Company | 24  | MJCF       |

### Humanoids

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `atlas_drc_description`       | Atlas DRC (v3)        | Boston Dynamics          | 30  | URDF       |
| `atlas_v4_description`        | Atlas v4              | Boston Dynamics          | 30  | URDF       |
| `draco3_description`          | Draco3                | Apptronik                | 25  | URDF       |
| `ergocub_description`         | ergoCub               | IIT                      | 57  | URDF       |
| `icub_description`            | iCub                  | IIT                      | 32  | URDF       |
| `jaxon_description`           | JAXON                 | JSK                      | 38  | URDF       |
| `jvrc_description`            | JVRC-1                | AIST                     | 34  | URDF       |
| `jvrc_mj_description`         | JVRC-1                | AIST                     | 34  | MJCF       |
| `r2_description`              | Robonaut 2            | NASA JSC Robotics        | 56  | URDF       |
| `romeo_description`           | Romeo                 | Aldebaran Robotics       | 37  | URDF       |
| `sigmaban_description`        | SigmaBan              | Rhoban                   | 20  | URDF       |
| `talos_description`           | TALOS                 | PAL Robotics             | 32  | URDF       |
| `valkyrie_description`        | Valkyrie              | NASA JSC Robotics        | 59  | URDF       |

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
| `tiago_description`           | TIAGo                 | PAL Robotics             | 48  | URDF       |

### Quadrupeds

| Name                          | Robot                 | Maker                    | DOF | Format     |
|-------------------------------|-----------------------|--------------------------|-----|------------|
| `a1_mj_description`           | A1                    | UNITREE Robotics         | 12  | MJCF       |
| `a1_description`              | A1                    | UNITREE Robotics         | 12  | URDF       |
| `b1_description`              | B1                    | UNITREE Robotics         | 12  | URDF       |
| `aliengo_description`         | Aliengo               | UNITREE Robotics         | 12  | MJCF, URDF |
| `anymal_b_mj_description`     | ANYmal B              | ANYbotics                | 12  | MJCF       |
| `anymal_b_description`        | ANYmal B              | ANYbotics                | 12  | URDF       |
| `anymal_c_mj_description`     | ANYmal C              | ANYbotics                | 12  | MJCF       |
| `anymal_c_description`        | ANYmal C              | ANYbotics                | 12  | URDF       |
| `go1_mj_description`          | Go1                   | UNITREE Robotics         | 12  | MJCF       |
| `go1_description`             | Go1                   | UNITREE Robotics         | 12  | URDF       |
| `hyq_description`             | HyQ                   | IIT                      | 12  | URDF       |
| `laikago_description`         | Laikago               | UNITREE Robotics         | 12  | MJCF, URDF |
| `mini_cheetah_description`    | Mini Cheetah          | MIT                      | 12  | URDF       |
| `minitaur_description`        | Minitaur              | Ghost Robotics           | 16  | URDF       |
| `solo_description`            | Solo                  | ODRI                     | 12  | URDF       |

## Contributing

New robot descriptions are welcome! Check out the [guidelines](https://github.com/robot-descriptions/robot_descriptions.py/tree/master/CONTRIBUTING.md) then open a PR.

## Thanks

Thanks to the maintainers of all the git repositories that made these robot descriptions available.

## See also

Robot descriptions in other languages:

| ![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=c%2B%2B&logoColor=white) | [robot\_descriptions.cpp](https://github.com/mayataka/robot_descriptions.cpp) |
|--|--|
