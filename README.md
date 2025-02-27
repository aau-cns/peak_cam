# peak_cam

ROS 2 version of the peak_cam package. The original package was developed for ROS 1 and can be found [here](https://github.com/SherifN/peak_cam)
A Linux [ROS C++ Node](https://wiki.ros.org/peak_cam) that wraps the driver API for IDS vision cameras using IDS peak software. Tested on Ubuntu 18.04 LTS.

## How to run

### Before running the code

1. install [ROS 2](https://docs.ros.org/en/jazzy/index.html)
2. install [IDS peak](https://de.ids-imaging.com/downloads.html)

### Running the code

1. Generate a ROS 2 workspace
    ```bash
    mkdir -p <path-to-workspace>/src/
    ```
2. Clone the repository into the workspace na dbuild the workspace
    ```bash
    cd <path-to-workspace>/src/
    git clone -b ros2 https://github.com/aau-cns/peak_cam
    cd ..
    colcon build --symlink-install
    ```
3. Set parameters such as ROS topic and acquisition rate under [`params/settings/peak_cam_params.yaml`](src/peak_cam/params/settings/peak_cam_params.yaml)
4. Plug the IDS vision camera into the device and launch the node 
    ```bash
    source install/setup.bash
    ros2 launch peak_cam peak_cam.launch.py
    ```
5. Stop the node with `Ctrl-C` (SIGINT) for controlled shutdown 

For multiple cameras, create a `.launch.py` and a `.yaml` file for each camera.

> Hint: Sometimes the cameras are only accesible as root. Try ` sudo -s` in your terminal and launch the node again.

Copyright (c) 2020, Sherif Nekkah

All rights reserved.

BSD license: see LICENSE file
