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