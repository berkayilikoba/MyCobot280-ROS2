# MyCobot280-ROS2

A workspace prepared to control the Elephant Robotics **myCobot 280** robot arm with **ROS 2 Humble** in simulation (Gazebo) and on real hardware.

---

## 1. Requirements

| Component | Version / Description |
| --- | --- |
| Operating System | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10+ |
| Robot | myCobot 280 (M5 / Pi) |

---

## 2. Installation and Build

1. Navigate to the `src` directory of your workspace and clone the repository:
```bash
cd ~/colcon_ws/src
git clone https://github.com/berkayilikoba/MyCobot280-ROS2.git

```


2. Install dependencies and build the workspace:
```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

```



---

## 3. Execution Commands

### A. Gazebo Simulation and Pick-and-Place

1. **Start the Simulation:**
```bash
ros2 launch mycobot_gazebo gazebo_pick.launch.py

```


2. **Start RViz Visualization:**
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix mycobot_gazebo)/share/mycobot_gazebo/rviz/pick_place.rviz --ros-args -p use_sim_time:=true

```


3. **Run the Pick and Place Script:**
```bash
ros2 run mycobot_gazebo cm

```



### B. Real Robot Test (`real_robot`)

1. Connect the robot via USB and grant serial port permissions:
```bash
sudo chmod 666 /dev/ttyUSB0

```


2. Run the test node:
```bash
ros2 run real_robot real_test

```



---

## 4. Project Structure

```text
MyCobot280-ROS2/
├── mycobot_gazebo/    # Gazebo simulation, launch, rviz, and Python scripts
└── real_robot/        # Real hardware control and test packages

```
