# xarm_ROS2_Ubuntu22.04_Humble

**Install python**
``` bash
sudo apt update
sudo apt install python3.10 python3.10-venv python3.10-dev
```
you can using `python3 --version` check version in your bash

**Create workspace**
```
cd ~
mkdir -p dev_ws/src
```

**Create `.venv**
```bash
cd ~/dev_ws/
python3.10 -m venv .venv
```

**activate `.venv`**
```bash
cd ~/dev_ws/
source .venv/bin/activate
```

## 1. Introduction

&ensp;&ensp;&ensp;&ensp;This repository contains simulation models, and corresponding motion planning and controlling demos of the xArm series from UFACTORY. The development and test environment is as follows
- Ubuntu 20.04 + ROS Foxy
- Ubuntu 20.04 + ROS Galactic
- Ubuntu 22.04 + ROS Humble
- Ubuntu 24.04 + ROS Jazzy
- Ubuntu 22.04 + ROS Rolling

&ensp;&ensp;&ensp;&ensp;Please switch to the corresponding code branch according to different ros2 versions (no corresponding code branch means it has not been tested in this version)
- Foxy: [foxy](https://github.com/xArm-Developer/xarm_ros2/tree/foxy)
- Galactic: [galactic](https://github.com/xArm-Developer/xarm_ros2/tree/galactic)
- Humble: [humble](https://github.com/xArm-Developer/xarm_ros2/tree/humble)
- Jazzy: [jazzy](https://github.com/xArm-Developer/xarm_ros2/tree/jazzy)
- Rolling: [rolling](https://github.com/xArm-Developer/xarm_ros2/tree/rolling)

## 2. Preparation

- ### 2.1 Install [ROS2](https://docs.ros.org/) 
  - [Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Installation.html)
  - [Galactic](https://docs.ros.org/en/ros2_documentation/galactic/Installation.html)
  - [Humble](https://docs.ros.org/en/ros2_documentation/humble/Installation.html)  ----------- Here I'm install -----------
  - [Jazzy](https://docs.ros.org/en/ros2_documentation/jazzy/Installation.html)

**Setting up your Ubuntu environment**
```bash
sudo apt update
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
**Enable necessary software sources**
``` bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
```
**setting source && GPG Key**
```bash 
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
**Install ROS 2 Humble**
```bash
sudo apt update
sudo apt upgrade -y

sudo apt install ros-humble-desktop -y
```
**Development Tools (optional)**
``` bash
sudo apt install ros-dev-tools -y
```

**Setting environment variables**
Enable only on the current terminal:
```bash
source /opt/ros/humble/setup.bash
```
**Automatically enable every time (recommended):**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Try examples:
Open two terminals:

**talker**
```
ros2 run demo_nodes_cpp talker
```
**listener:**
```
ros2 run demo_nodes_cpp listener
```

### 2.2 Install [Moveit2](https://moveit.ros.org/install-moveit2/binary/)
```
sudo apt install ros-humble-moveit
```

### Install Tips [URL](https://gazebosim.org/docs/all/getstarted/)
[Summary of Compatible ROS and Gazebo Combinations](https://gazebosim.org/docs/latest/ros_installation/)

### 2.3 Install [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)


#### if you install ROS 2 Jazzy - ******************-------------you will found it is not useful-------------******************

1. 安裝必要的工具：
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg curl
```
2. 添加 Gazebo 的官方 GPG 金鑰：
```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
```
3. 添加 Gazebo 的套件庫到系統來源列表：
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```
4. 更新套件列表並安裝 Gazebo Ionic：
```bash
sudo apt-get update
sudo apt-get install gz-ionic
```

安裝完成後，你就可以使用最新版本的 Gazebo 模擬器了！🚀
```
gz sim
```

這些步驟是根據 Gazebo 官方的安裝指南整理 [Gazebo 官方安裝指南](https://gazebosim.org/docs/latest/install_ubuntu/)

**Tips:**
- Gazebo 的版本更新： Gazebo 團隊最近對版本命名進行了調整，將新版本命名為 Gazebo Ionic，而不是之前的 Garden。因此，套件名稱也相應地更新了。 [Gazebo 官方公告](https://gazebosim.org/docs/latest/getstarted/)
  
- ROS 2 的整合： 如果你同時使用 ROS 2 Jazzy，Gazebo Ionic 與其有良好的整合性，可以提供更順暢的開發體驗。 [ROS 2 與 Gazebo 的整合](https://github.com/gazebosim/docs/blob/master/ros_installation.md)

### 2.4 Install [ros_gz](https://github.com/gazebosim/ros_gz) - ROS 2 ↔ Gazebo Sim Bridge

  Since we are using **Gazebo Sim (Ionic)** instead of Gazebo Classic,  
we need to use the new bridge package `ros_gz` to integrate with ROS 2.

First you need to check you GZ version
```bash
gz sim --version
```
**now you have two Options**

**Option 1 : Switch to Gazebo Fortress (recommended)**
For the best stability and official support, it is recommended that you use Gazebo Fortress, which is the version of Gazebo most compatible with ROS 2 Humble.

1. Remove the existing Gazebo Harmonic (if installed):
```bash
sudo apt remove gz-harmonic
sudo apt remove 'gz-*' 'libgz-*' 'python3-gz-*' -y
sudo apt autoremove -y
```

2. install Gazebo Fortress：
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg curl
```
3. Download and install OSRF's GPG:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
```
4. Add the Gazebo repository to your system:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

```
5. Install the ros_gz package:
```
sudo apt-get update
sudo apt-get install ignition-fortress
```

6. test
```bash
ign gazebo
```
...or 
```bash
ign gazebo empty.sdf
```

7. Checking the Installed Version
```bash
ign gazebo --version
```
---
---
**Option 2 : Continue to use Gazebo Harmonic and compile ros_gz from source**
1. Setting environment:
```bash
export GZ_VERSION=harmonic
```

2. Create a workspace and download the source code:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble https://github.com/gazebosim/ros_gz.git
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src -i -y --rosdistro humble
```

4. Compiler Kit:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
5. Set environment variables:
```bash
source install/setup.bash
```

**More info:**
- [ros_gz GitHub Repository](https://github.com/gazebosim/ros_gz)
- [Gazebo ROS 2 Integration Guide](https://gazebosim.org/docs/latest/ros_installation/)

---

### 💬 Bonus 說明
**Note:** 
`gazebo_ros_pkgs` is used for **Gazebo Classic (e.g. gazebo11)**.  
Since this project uses `gz-ionic` (Gazebo Sim), please use `ros_gz` for ROS integration.


---
## How To Use

### 3.1 Create a workspace
Create a workspace - Skip this step if you already have a target workspace
```bash
cd ~
mkdir -p dev_ws/src
```

### 3.2 Obtain source code of "xarm_ros2" repository
```bash
# Remember to source ros2 environment settings first
cd ~/dev_ws/src
# DO NOT omit "--recursive"，or the source code of dependent submodule will not be downloaded.
# Pay attention to the use of the -b parameter command branch, $ROS_DISTRO indicates the currently activated ROS version, if the ROS environment is not activated, you need to customize the specified branch (foxy/galactic/humble/jazzy)
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
```

### 3.3 Update "xarm_ros2" repository
```bash
cd ~/dev_ws/src/xarm_ros2
git pull
git submodule sync
git submodule update --init --remote
```

### 3.4 Install dependencies
```bash
cd ~/dev_ws/
source .venv/bin/activate
pip install rosdep
pip install jinja2 typeguard

# Remember to source ros2 environment settings first
cd ~/dev_ws/src/
sudo $(which rosdep) init
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

### 3.5 Build xarm_ros2
```bash
deactivate
rm -rf build install log
colcon build

source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-empy -y
pip install numpy
pip install lark

# Remember to source ros2 and moveit2 environment settings first
cd ~/dev_ws/
# build all packages
colcon build

# build selected packages
colcon build --packages-select xarm_api
```

## Using Xarm

```bash
cd ~/dev_ws/
source install/setup.bash
```

...or
```bash
echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

install env
```
sudo apt install python3-lxml
```
run 
```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true
```
to start Gazebo and RViz

run with real robot
```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true robot_ip:=
```

using camera `add_realsense_d435i:=true`

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true add_realsense_d435i:=true
```

fake camera

```bash
ros2 run rqt_image_view rqt_image_view
```
**重要！請從下拉選單選擇：**
`/image_raw`



real camera

To see what divice you have:
```
v4l2-ctl --list-devices
```

你會看到像：
```bash
UVC Camera (046d:081b):
    /dev/video2
HP Wide Vision HD Camera:
    /dev/video0
```
install v4l2_camera node:
```bash
sudo apt install ros-${ROS_DISTRO}-image-transport-plugins
sudo apt install ros-${ROS_DISTRO}-v4l2-camera
```



To see your image:
```bash
ros2 run v4l2_camera v4l2_camera_node
ros2 run rqt_image_view rqt_image_view
```

**重要！請從下拉選單選擇：**
`/image_raw`

### Try to using the python code

