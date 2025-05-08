# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

## [1.17.0] - 2025-05-08

### Added

- Description: Adam Lite (MJCF)
- Description: ARX L5 (MJCF) (thanks to @jonzamora)
- Description: Kinova Gen3 Lite (URDF)
- Description: Low-cost robot arm (MJCF)

### Changed

- Update `baxter_common` repository with a commit ID rather than a tag
- Update `talos-data` repository with a commit ID rather than a tag

## [1.16.0] - 2025-04-09

### Added

- Add tags attribute to Description class
- Description: Ability Hand (MJCF)
- Description: Ability Hand (URDF)
- Description: Apptronik Apollo (MJCF)
- Description: BXI Elf2 (MJCF)
- Description: BXI Elf2 (URDF)
- Example: Load all humanoid robot descriptions
- Example: Load all quadruped robot descriptions
- Example: Show any robot description in the Candlewick viewer
- Export `DESCRIPTIONS` dictionary from top-level module

### Changed

- CICD: Update checkout actions to v4
- Update MuJoCo Menagerie repository commit
- Update eDO description repository
- Update ergoCub description model to SN002 (thanks to @traversaro)
- Update ergocub-software repository to v0.7.7 (thanks to @traversaro)
- Update simple-humanoid-description repository commit (thanks to @drfenixion)

## [1.15.0] - 2025-03-06

### Added

- Description: AgileX PiPER (MJCF)
- Description: AgileX PiPER (URDF)
- Description: ANYmal D
- Description: Robot Soccer Kit
- Description: Robotiq 2F-85 (MJCF V4) (thanks to @peterdavidfagan)
- CLI: Add `show_in_meshcat` command
- CLI: Add `show_in_mujoco` command
- CLI: Add `show_in_pybullet` command

### Changed

- Enable command-line to run from `python -m robot_descriptions`
- CLI: Rename `show` command to `show_in_yourdfpy`

### Fixed

- minor: Use single backticks for inline command-line instructions

## [1.14.0] - 2025-01-07

### Added

- Description: SO-ARM100 (MJCF) (thanks to @JafarAbdi)
- Description: Berkeley Humanoid

### Fixed

- cache: Only call git checkout when it is not head of the working copy
- Example: simulated loaded robot description in PyBullet

## [1.13.0] - 2024-10-30

### Added

- Description: Left and right LEAP hand (MJCF) (thanks to @kevinzakka)
- Description: PAL Talos (MJCF) (thanks to @kevinzakka)
- Description: SO-ARM100
- Description: Unitree B2

### Changed

- Upkie: Update repository to v2.1.0

## [1.12.0] - 2024-08-08

### Added

- Description: LEAP Hand v1
- Description: Shadow DEX-EE (MJCF) (thanks to @kevinzakka)
- Description: Stretch 3 (MJCF) (thanks to @kevinzakka)
- Pinocchio: load MJCF robot descriptions (thanks to @lvjonok)

### Changed

- Update repository for `gen3_mj_description` to `mujoco_menagerie`

## [1.11.0] - 2024-06-27

### Added

- Description: KUKA iiwa 7 (URDF)
- Description: Unitree G1 (MJCF) (thanks to @kevinzakka)
- Description: Unitree G1 (URDF) (thanks to @lvjonok)
- Description: Boston Dynamics Spot (MJCF) (thanks to @kevinzakka)
- Description: Franka Robotics FR3 (MJCF) (thanks to @kevinzakka)
- Description: WidowX 250 6-DOF (MJCF) (thanks to @kevinzakka)

### Changed

- Bump minimum Python version to 3.9
- CICD: Pin NumPy version to <2 for PyBullet and yourdfpy
- CICD: Update ruff to 0.2.2
- Rename `iiwa_description` to `iiwa14_description`
- Rename `iiwa_mj_description` to `iiwa14_mj_description`
- Update repository for `iiwa14_description` to `drake_models`

## [1.10.0] - 2024-04-24

### Added

- Description: UFACTORY xArm7 (MJCF) (thanks to @kevinzakka)
- Variant keyword in the MuJoCo loader (thanks to @fabinsch)

### Changed

- Split Aliengo into MJCF and URDF descriptions (thanks to @Danfoa)
- Update Aliengo URDF to a more active repository (thanks to @Danfoa)

## [1.9.0] - 2024-03-27

### Added

- Citation file and BibTeX section in the readme
- Description: Allegro Hand V3 (MJCF) (thanks to @kevinzakka)
- Description: ALOHA 2 (MJCF) (thanks to @kevinzakka)
- Description: Bitcraze Crazyflie 2.0 (MJCF) (thanks to @kevinzakka)
- Description: KUKA iiwa 14 (MJCF) (thanks to @kevinzakka)
- Description: Robotics OP3 (MJCF) (thanks to @kevinzakka)
- Description: Saywer (MJCF) (thanks to @kevinzakka)
- Description: Stretch 2 (MJCF) (thanks to @kevinzakka)
- Description: ViperX 300 6DOF (MJCF) (thanks to @kevinzakka)

### Changed

- Go1: Switch repository to MuJoCo Menagerie
- Go2: Switch repository to MuJoCo Menagerie
- H1: Switch repository to MuJoCo Menagerie
- JVRC-1: Update repository to maintained one at jrl-umi3218
- Upkie: Update repository to v1.5.0

### Fixed

- Bug when cloning a non-pinned description after a commit-pinned one
- Package name resolution when pinning a description to a specific commit

## [1.8.1] - 2024-01-29

### Changed

- Update example-robot-data to version 4.0.9 (thanks to @htadashi)

## [1.8.0] - 2024-01-24

### Added

- Description: UNITREE Z1 (thanks to @lvjonok)
- Description: Skydio X2 (thanks to @lvjonok)

### Changed

- Bump yourdfpy minimum version to 0.0.56

### Fixed

- Path to UNITREE H1 (thanks to @lvjonok)
- Unit testing on UNITREE H1

## [1.7.0] - 2023-12-08

### Added

- Description: UNITREE Go2 (MJCF) (thanks to @lvjonok)
- Description: UNITREE Go2 (thanks to @lvjonok)
- Description: UNITREE H1 (MJCF) (thanks to @lvjonok)
- Description: UNITREE H1 (thanks to @lvjonok)
- Description: UR10e (MJCF)

## [1.6.0] - 2023-05-23

### Added

- Description: FANUC M-710iC
- Description: Gen3
- Description: TriFingerEdu

### Changed

- Update Stretch description to v1.0.0

## [1.5.0] - 2023-04-13

### Added

- Description: Rhea
- Description: Stretch
- Frame selector in URDF frame display example
- README: Conda installation instructions

### Changed

- Update Upkie description to v1.2.0

## [1.4.1] - 2023-02-28

### Added

- Description: Draco3 (thanks to @shbang91)
- Description: ergoCub

### Changed

- CI: switch to ruff

## [1.4.0] - 2023-02-08

### Added

- Ability to load a robot description at a specific commit
- Example: display all frames of a URDF description

### Fixed

- Recover from empty or invalid cache git repositories

## [1.3.1] - 2023-01-13

### Added

- Description: B1
- Description: Z1

### Changed

- A1: use original URDF from Unitree
- Updated `unitree_ros` repository

## [1.3.0] - 2022-12-16

### Changed

- iDynTree: loader now supports package directories.
- Update example-robot-data repository to v4.0.3

### Fixed

- CI: check code style

## [1.2.0] - 2022-12-07

### Added

- Loader: iDynTree

## [1.1.0] - 2022-11-29

### Added

- Description: Spryped
- Loader: RoboMeshCat

### Changed

- CI: run loader tests in separate jobs (avoids a memory limit)
- Command-line: only require yourdfpy for show command
- Pinocchio: bump minimum supported version to 2.6.10

### Fixed

- CI: remove exceptions for bugs that have been fixed upstream
- Examples: remove exception on Crazyflie 2.0 description as it has been fixed

## [1.0.0] - 2022-10-31

### Added

- Description: JAXON
- Description: Kinova Gen3
- Description: NEXTAGE
- Description: SigmaBan
- Unit tests for repository cloning

### Changed

- MuJoCo examples: try `mj_description` suffix rather than `description`
- Update Cassie URDF description to an MIT-licensed repository

### Fixed

- Shadow Hand MJCF: upstream fix to joint axes
- Update cached repository after a description was updated

## [0.6.0] - 2022-09-28

### Added

- CI: check that all URDF descriptions load in yourdfpy
- Example: load in yourdfpy
- Loader: yourdfpy

## [0.5.0] - 2022-09-19

### Added

- CI: check that all MJCF descriptions load in MuJoCo
- Description: BarrettHand
- Description: Eve R3
- Description: Robonaut 2
- Description: Valkyrie
- Loader: MuJoCo
- Unit tests to check that MuJoCo loads all MJCF descriptions successfully

### Changed

- Cassie: update to description from MuJoCo Menagerie
- Pinocchio loader: no root joint by default
- Update MuJoCo Menagerie repository to propagate fix

## [0.4.0] - 2022-09-14

### Added

- CI: check that all URDF descriptions load in PyBullet
- Description: Poppy Ergo Jr
- Description: Poppy Torso
- Loader: PyBullet
- Show description formats in `robot_descriptions list`
- Unit tests to check that Pinocchio loads all descriptions successfully
- Unit tests to check that PyBullet loads all descriptions successfully

### Changed

- A1: Split description again into URDF and MJCF again
- Command-line: refer to yourdfpy for animation
- eDO: update cache path to match package name
- Flatten cache directory structure
- Ginger: update cache path to match package name
- Go1: Split description again into URDF and MJCF again
- Match cache path with package name when it helps PyBullet
- Remove `MESHES_PATH` attribute

## [0.3.0] - 2022-09-12

### Added

- CI: check that URDF descriptions load in Pinocchio
- Description: ANYmal B MJCF
- Description: ANYmal C MJCF
- Description: Atlas v4
- Description: Fetch
- Description: Ginger
- Description: Go1
- Description: Laikago MJCF
- Description: Panda MJCF
- Description: Pepper
- Description: Robotiq 2F-85
- Description: Robotiq 2F-85 MJCF
- Description: Shadow Hand MJCF
- Description: UR5e MJCF
- Description: YuMi
- Loader: Pinocchio
- Unit test fixture for loaders

### Changed

- Merge MJCF and URDF descriptions for Aliengo
- Renamed former "Atlas" description to "Atlas DRC"
- Switch to a dual MJCF/URDF descriptions for A1

## [0.2.0] - 2022-09-07

### Added

- Description: Aliengo MJCF
- Description: Cassie MJCF
- Description: JVRC-1 MJCF
- Example: load in MuJoCo
- Example: load in Pinocchio
- Example: load in PyBullet
- Example: show in MeshCat
- Example: show in MuJoCo
- Example: show in PyBullet
- Example: show in yourdfpy
- New `REPOSITORY_PATH` member for each description
- Support MJCF descriptions

### Changed

- `PATH` becomes `PACKAGE_PATH`

### Fixed

- iCub description

## [0.1.1] - 2022-09-06

### Fixed

- Command line usage instructions
- Upload coverage results based on `USING_COVERAGE` setting

## [0.1.0] - 2022-09-05

This initial release includes 33 robot descriptions:

- A1
- Aliengo
- Allegro Hand
- ANYmal B
- ANYmal C
- Atlas
- Baxter
- Bolt
- Cassie
- Crazyflie 2.0
- Double Pendulum
- e.DO
- FingerEdu
- Gen2
- HyQ
- iCub
- iiwa 14
- JVRC-1
- Laikago
- Mini Cheetah
- Minitaur
- Panda
- PR2
- Reachy
- Romeo
- Simple Humanoid
- Solo
- TALOS
- TIAGo
- Upkie
- UR10
- UR3
- UR5

### Added

- Caching of git repositories
- Command line to show or animate robot descriptions
- Contributing instructions
- This changelog

[unreleased]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.17.0...HEAD
[1.17.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.16.0...v1.17.0
[1.16.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.15.0...v1.16.0
[1.15.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.14.0...v1.15.0
[1.14.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.13.0...v1.14.0
[1.13.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.12.0...v1.13.0
[1.12.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.11.0...v1.12.0
[1.11.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.10.0...v1.11.0
[1.10.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.9.0...v1.10.0
[1.9.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.8.1...v1.9.0
[1.8.1]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.8.0...v1.8.1
[1.8.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.7.0...v1.8.0
[1.7.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.6.0...v1.7.0
[1.6.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.5.0...v1.6.0
[1.5.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.4.1...v1.5.0
[1.4.1]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.4.0...v1.4.1
[1.4.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.3.1...v1.4.0
[1.3.1]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.2.0...v1.3.0
[1.2.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v0.6.0...v1.0.0
[0.6.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/robot-descriptions/robot_descriptions.py/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/robot-descriptions/robot_descriptions.py/releases/tag/v0.1.0
