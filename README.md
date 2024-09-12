# Barrett-7dof-WAM-gazebo-moveit

# Build

## 安装所需的ros库：
sudo apt install ros-melodic-joint-state-controller ros-melodic-moveit  ros-melodic-robot-state-publisher  ros-melodic-effort-controllers

## 创建工作空间：
mkdir ws_wam_arm  && cd ws_wam_arm && mkdir src  

## 下载：
cd src
git clone https://github.com/TINY-KE/Barrett-7dof-WAM-arm-gazebo-moveit.git

## 编译
cd ..
catkin_make


# run
## 添加工作空间的路径
source devel/setup.bash 

## 启动仿真环境（末端为抓手）
roslaunch    wam_arm_moveit    wam_hand_bringup_moveit_onlyrobot.launch 

## 启动仿真环境（末端为相机）
roslaunch    wam_arm_moveit    wam_kinectv1_bringup_moveit_onlyrobot.launch 