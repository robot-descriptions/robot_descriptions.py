#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Giulio Romualdi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os.path
from typing import List


def get_package_dirs(module) -> List[str]:
    """Get package directories for a given module.

    Args:
        module: Robot description module.

    Returns:
        Package directories.
    """
    return [
        module.PACKAGE_PATH,
        module.REPOSITORY_PATH,
        os.path.dirname(module.PACKAGE_PATH),
        os.path.dirname(module.REPOSITORY_PATH),
        os.path.dirname(module.URDF_PATH),  # e.g. laikago_description
    ]
