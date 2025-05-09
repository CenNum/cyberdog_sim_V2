#!usr/bin/env python

# Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import time

def launchsim():

    os.system('gnome-terminal -t "cyberdog_gazebo" -e "bash ./src/cyberdog_simulator/cyberdog_gazebo/script/launchgazebo.sh"')
    time.sleep(5)
    os.system('gnome-terminal -t "cyberdog_viusal" -e "bash ./src/cyberdog_simulator/cyberdog_gazebo/script/launchvisual.sh"')

    os.system('gnome-terminal -t "cyberdog_control" -e "bash ./src/cyberdog_simulator/cyberdog_gazebo/script/launchcontrol.sh"')

if __name__=="__main__":
    launchsim()
