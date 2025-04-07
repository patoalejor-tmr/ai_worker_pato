# 🦾 FFW-1

## **1. Introduction**

This is the **FFW (Freedom From Work)** project repository. It supports the control of the **Follower** device (hand, lift, and neck) as well as the **Leader** device (arms and hands).

This package supports **ROS 2 Jazzy** and **Gazebo Harmonic** on Ubuntu 24.04 **OpenMANIPULATOR User Guide**



## **2. Prerequisites**

### **Supported ROS Version**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)

This package is compatible only with **ROS 2 Jazzy**. Ensure that ROS 2 Jazzy is properly installed.

### **Required Packages**

Install the following dependencies:

```bash
sudo apt-get update && sudo apt-get install -y \
    libboost-all-dev \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-tf-transformations \
    ros-jazzy-gz* \
    ros-jazzy-pal-statistics \
    python3-tk
sudo apt-get install -y ros-jazzy-moveit-* --no-install-recommends
```

### **USB Port Permissions**

To enable communication with the hardware, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

**A login and logout are required.**



## **3. Installation**

### **1. Clone the Repository**

Navigate to your ROS 2 workspace and clone the repository:

```bash
cd ~/${WORKSPACE}/src
```

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
```

### **2. Build the Package**

Compile the package using `colcon`:

```bash
cd ~/${WORKSPACE}
colcon build --symlink-install
```

### **3. Source the Workspace**

```bash
source ~/${WORKSPACE}/install/setup.bash
```

Create and apply `udev` rules:

```bash
ros2 run ffw_bringup ffw_create_udev_rules
```


## **4. Execution Commands**

### **Step 1: Choose Your Operating Mode**

#### **0️⃣ Calibration**
When using it for the first time, wear the leader device and perform calibration to check the range of motion.
This process is done only once initially.

```bash
ros2 launch ffw_bringup hand_calibration.launch.py
```

#### **1️⃣ Leader-Follower Mode**

For **leader-follower functionality**, use:

```bash
ros2 launch ffw_bringup ffw_teleop_with_hand.launch.py
```

Ensure proper connection and detection of leader and follower devices.

**Operation Sequence upon Launch**

1. Establish hardware connection with the ***follower*** device.
2. Move the *follower* device to the **initial position**.( Standing position with arms outstretched)
3. Establish hardware connection with the ***leader*** device.

#### **2️⃣ Standalone Mode**

For **standalone mode**, launch:

```bash
ros2 launch ffw_bringup hardware_follower_teleop_with_hand.launch.py
```

*Only the follower is connected to the hardware interface.



```
ros2 launch ffw_bringup hardware_leader_with_hand.launch.py
```

*Only the Leader is connected to the hardware interface.



```
ros2 launch ffw_bringup hardware_follower_standalone.launch.py
```

*Mode to operate the follower via MoveIt

#### **3️⃣ Gazebo Simulation Mode**

For **Gazebo simulation mode**, launch:

```bash
ros2 launch ffw_bringup gazebo.launch.py
```

Ensure that Gazebo Harmonic is properly installed and configured before running the simulation.


### **Step 2: Extend Functionality**

#### **1. MoveIt! Launch**

Enable MoveIt functionality for advanced motion planning in RViz:
*Before operation, either `hardware_follower_standalone.launch.py` or Gazebo must be launched."

```bash
ros2 launch ffw_moveit_config moveit_core.launch.py
```

Move interactive markers to position the robotic arm, then click **Plan** and **Execute**.

#### **2. GUI Teleop**

```bash
ros2 launch ffw_teleop keyboard_control_teleop_with_hand.launch.py
```

This is for `hardware_follower_teleop_with_hand.launch.py` and `hardware_follower_teleop_without_hand.launch.py`



```
ros2 launch ffw_teleop keyboard_control_standalone.launch.py
```

This is for `hardware_follower_standalone.launch.py`

