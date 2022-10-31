# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2022/10/31

### Added

- Description: JAXON
- Description: Kinova Gen3
- Description: NEXTAGE
- Description: SigmaBan
- Unit tests for repository cloning

### Changed

- MuJoCo examples: try ``mj_description`` suffix rather than ``description``
- Update Cassie URDF description to an MIT-licensed repository

### Fixed

- Shadow Hand MJCF: upstream fix to joint axes
- Update cached repository after a description was updated

## [0.6.0] - 2022/09/28

### Added

- CI: check that all URDF descriptions load in yourdfpy
- Example: load in yourdfpy
- Loader: yourdfpy

## [0.5.0] - 2022/09/19

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

## [0.4.0] - 2022/09/14

### Added

- CI: check that all URDF descriptions load in PyBullet
- Description: Poppy Ergo Jr
- Description: Poppy Torso
- Loader: PyBullet
- Show description formats in ``robot_descriptions list``
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
- Remove ``MESHES_PATH`` attribute

## [0.3.0] - 2022/09/12

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

## [0.2.0] - 2022/09/07

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
- New ``REPOSITORY_PATH`` member for each description
- Support MJCF descriptions

### Changed

- ``PATH`` becomes ``PACKAGE_PATH``

### Fixed

- iCub description

## [0.1.1] - 2022/09/06

### Fixed

- Command line usage instructions
- Upload coverage results based on ``USING_COVERAGE`` setting

## [0.1.0] - 2022/09/05

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
