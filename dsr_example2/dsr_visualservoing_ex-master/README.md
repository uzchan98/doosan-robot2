# Overview
    
    This package provides the visual servoing example using Doosan robots in the ROS2(Humble) environment.
    
    ※ Currently, ROS2 related packages are being updated rapidly. 
       Doosan packages will also be updated from time to time and features will be upgraded.


---
# Prequiste List
## To run the visual servoing example, Have to download these packages and library.

1. [doosan-robot2 (rt-feature branch)](https://github.com/doosan-robotics/doosan-robot2/tree/rt_feature)

2. [ros-gz: gazebo (humble branch)](https://github.com/gazebosim/ros_gz/tree/humble) at humble

3. `tf-transformations`

```bash
pip3 install tf-transformations
```

---
# How to install `dsr_visualservoing_ex` package.
1. Move to `doosan-robot2` package.
```bash
cd ~/ros2_ws/src/doosan-robot2
```

2. Clone `dsr_visualservoing_ex` package.
```bash
git clone 
```

3. Install dependency
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace
```bash
colcon build
source install/setup.bash
```

## Directory Structure after Build:
```bash
├── user
    ├── ros2_ws
        ├── build
        ├── install
        ├── log
        ├── src
            ├── doosan-robot2
            │   ├── common2
            │   ├── dsr_bringup2
            │   ├── dsr_controller2
            │   ├── dsr_description2
            │   ├── dsr_gazebo2
            │   ├── dsr_hardware2
            │   ├── dsr_moveit2
            │   ├── dsr_msgs2
            │   ├── dsr_visualservoing_ex
            │   ├── install_emulator.sh
            │   ├── LICENSE
            │   ├── README.md
            │   ├── uninstall_emulator.sh
            ├── gz_ros2_control
            ├── ros_gz
```


---
# How to run visual servoing example
1. Start the Simulation
```bash
ros2 launch dsr_visualservoing_ex dsr_visual_servoing_gazebo.launch.py
```

2. Move to the home position
```bash
ros2 run dsr_visualservoing_ex joint90
```

3. Run the visual servoing example, when the cobot reaches the home position ([0, 0, 90, 0, 90, 0]).
```bash
ros2 launch dsr_visualservoing_ex  visual_servoing_gz.launch.py
```

---
# Explain the `dsr_visualsesrvoing_ex` package
## 1.`dsr_bringup2_visual_servoing_gazebo.launch.py`
Explain:
- This is a launch file that utilizes `dsr_bringup2_gazebo.launch.py`.
- It sets up a simulation environment for Visual Servoing using the `visual_servoing.sdf` file.
- The `rgbd_camera.launch.py` file is used to enable the RGB-D camera in Gazebo.
    - The installation of the Gazebo tutorial package `ros_gz` is required.
    - This is a launch file that uses `ros_gz/ros_gz_sim_demos/launch/image_bridge.launch.py`.
- The cobot is uploaded into Gazebo, making it controllable.


## 2.`joint90.py`
This is a node that moves the joints to the home position ([0, 0, 90, 0, 90, 0]) using the `/dsr01/motion/move_joint` service to prevent singularity during cobot control.

Service:
`/dsr01/motion/move_joint`


## 3. `visual_servoing_gz.launch.py`
Caution:
- Please run this launch file, when the cobot reaches the home position ([0, 0, 90, 0, 90, 0]).

### 3-1. `detect_marker_gz.py`
Explain:
- The RGB-D camera is used to recognize one ArUco marker at a time.
- The axes drawn on the ArUco Marker are based on the camera coordinate. 
- The coordinates of the recognized ArUco marker are transformed into the cobot's coordinate system using the Homogeneous Transformation Matrix calculation, and the `/marker/pose` ([x, y, z, rx, ry, rz]) values are published.
- If a new marker is not detected, the cobot moves to the home position ([0.0, 0.0, 90.0, 0.0, 90.0, 0.0]) using the `/dsr01/servoj_stream` topic.

Publish:
`/marker/id`
`/marker/pose`
`/dsr01/servoj_stream`

Subscribe:
`/dsr01/gz/joint_states`


### 3-2. `send_pose_servol_gz.py`
Explain:
- It subscribes to `/marker/id` and `/marker/pose`, which are published by `detect_marker_gz.py`.
- It checks whether dtecting marker.
- If the marker is not detected, it disconnects from the `/dsr01/servol_stream` topic.
- If the marker is detected, it uses the `/marker/pose` values as the `pos` values for the `/dsr01/servol_stream` topic to move the cobot to a position facing the ArUco marker.

Subscribe:
`/marker/id`
`/marker/pose`

Publish:
`/dsr01/servol_stream`


---
# Option

## + File to modify when changing camera performance:
The camera performance in visual_servoing.sdf needs to be changed to match the performance of the camera you want to use.

## + Using depth data:
To use depth data, you can utilize the `/rgbd_camera/depth_image` topic.

## + Camera Calibration Method:
1. Modify the `camera_calibration.sdf` file used in `camera_calibration.launch.py`.
It needs to be adjusted to match the performance of the camera being used.

2. Install the necessary packages for camera calibration:
```bash
sudo apt install ros-humble-camera-calibration
sudo apt install ros-humble-image-pipeline
```

3. Run `camera_calibration.launch.py`:
```bash
ros2 launch dsr_visualsercoing_ex camera_calibration.launch.py
```

4. Run the Camera Calibration Node:
```bash
ros2 run camera_calibration cameracalibrator --pattern chessboard --size 6x8 --square 0.02 image:=/rgbd_camera/image
```

5. Rotate and move the checkerboard uploaded in Gazebo until the calibration button becomes active.

6. After pressing the calibration button, click the save button and apply the results shown in the terminal (where the Camera Calibration Node was run) to the `rgbd_camera_gz.yaml` file.


---
# Caution
### 1. If the RAM capacity is low, build errors may occur, so please increase the memory capacity by setting up swap.

### 2. If you want to run Gazebo faster, then you should modify `dsr_controller2_gz.yaml`.
(Path: `doosan-robot2/dsr_controller2/config/dsr_controller2_gz.yaml`)






