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


### Step 0 🧹 先移除錯誤安裝（第一次建議完整清乾淨）

```bash
sudo apt remove '^ros-humble-*' 'gz-*' 'libgz-*' 'python3-gz-*' 'ignition-*' 'libignition*' --purge -y
sudo apt autoremove -y
sudo rm -rf ~/.ros ~/dev_ws /opt/ros/humble
```

### Step 1 🐢 安裝 ROS 2 Humble
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-humble-desktop -y
```

🧩 安裝開發工具
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete python3-vcstool ros-dev-tools
sudo apt install -y python3-empy
```

🔧 初始化 rosdep
```bash
sudo rosdep init  # 只執行一次
rosdep update
```

### Step 2 🤖 安裝 Gazebo Classic（xArm 官方支援的版本）
```bash 
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### Step 3 🏗️ 建立開發環境
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws
python3 -m venv .venv
source .venv/bin/activate
```

### Step 4 🔽 複製 xArm 原始碼
```bash 
cd ~/dev_ws/src
# 請確認有加上 --recursive 和 -b humble
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b humble

cd xarm_ros2
# 更新子模組
git submodule sync
git submodule update --init --remote
```

### Step 5 📦 安裝相依套件
```bash
source /opt/ros/humble/setup.bash
cd ~/dev_ws
pip install -U pip
pip install rosdep jinja2 typeguard lxml numpy==1.24.4 empy==3.3.4 lark-parser

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

### Step 6 🔨 編譯專案
```bash
colcon build --symlink-install
```
如果編譯失敗：
```bash
rm -rf build install log
colcon build --symlink-install
```
### Step 7 啟動模擬環境
✅ 設定環境變數

建議寫進 `.bashrc`：
```bash 
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

✅ 啟動 xArm + MoveIt + Gazebo（含夾爪）
```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true
```

------
------

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

...or adding camera in your .world
```world
<model name="top_down_camera">
  <static>true</static>
  <link name="camera_link">
    <pose>0 0 1.5 0 1.57 0</pose> <!-- 高度 1.5m，往下看 -->
    <sensor name="top_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <cameraName>camera</cameraName>
        <imageTopicName>/camera/image_raw</imageTopicName>
        <cameraInfoTopicName>/camera/camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </link>
</model>
```

and using 
```
ros2 run rqt_image_view rqt_image_view
```

**重要！請從下拉選單選擇：**
`/image_raw`



<!-- 
**real camera**

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
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
ros2 run rqt_image_view rqt_image_view
```

**重要！請從下拉選單選擇：**
`/image_raw`

-->


### Try to using the python code

### 如果要讓 gazebo 環境有物理效果 則須 改程式碼
- `/home/{**user_name**}/dev_ws/src/xarm_ros2/xarm_gazebo/launch/_robot_beside_table_gazebo.launch.py`
- `/home/{**user_name**}/dev_ws/src/xarm_ros2/xarm_moveit_config/launch/_robot_moveit_gazebo.launch.py`

run in bash 
```
vim /home/{**user_name**}/xarm_world_with_ground.world
```

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://table</uri>
      <pose>-0.3 -0.8 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

**test_your_world**
`gazebo /home/ckall123/xarm_world_with_ground.world`

```bash
colcon build --packages-select xarm_moveit_config xarm_gazebo
source ~/dev_ws/install/setup.bash

# 測試載入你的世界
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true world:=/home/{user_name}/xarm_world_with_ground.world  
# /home//.gazebo/models/xarm_world_with_ground.world
```
change your object in `/home/{**user_name**}/.gazebo/models/`

## 控制夾爪黏住東西
clone this github [https://github.com/IFRA-Cranfield/IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher)
you need to clone in you workspace 
```
cd ~/dev_ws/src
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
cd ~/dev_ws
# colcon build
colcon build --packages-select linkattacher_msgs ros2_linkattacher
```
你的Plugin .so 位置：`~/dev_ws/install/ros2_linkattacher/lib/libgazebo_link_attacher.so`
加 GAZEBO_PLUGIN_PATH（寫進 ~/.bashrc 或手動加）：
```
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/dev_ws/install/ros2_linkattacher/lib' >> ~/.bashrc
source ~/.bashrc
```
驗證設定
```
echo $GAZEBO_PLUGIN_PATH
```

.world 裡加這行 plugin：
```
<plugin name="gazebo_link_attacher" filename="libgazebo_link_attacher.so"/>
```
### 讓夾爪被夾起來
```
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink \
"{model1_name: 'UF_ROBOT', link1_name: 'right_finger', model2_name: 'beer_can', link2_name: 'body'}"
```
### 讓夾爪放下
```
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink \
"{model1_name: 'UF_ROBOT', link1_name: 'right_finger', model2_name: 'beer_can', link2_name: 'body'}"
```

Spawn 物件進入場景（如果不是 world 啟動時載入）
```
ros2 run gazebo_ros spawn_entity.py \
  -file /path/to/beer_can.sdf \
  -entity beer_can
```

------

```
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true world:=/home/ckall123/.gazebo/models/xarm_world_with_ground.world
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true world:=/home/m416-3090ti/dev_ws/world/xarm_world_with_ground.world
```

```bash
ros2 run rqt_image_view rqt_image_view
```
