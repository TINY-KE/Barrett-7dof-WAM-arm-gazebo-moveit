# Barrett-7dof-WAM-gazebo-moveit

## 一、编译Build

### 安装所需的ros库：
```
sudo apt install ros-melodic-joint-state-controller ros-melodic-moveit  ros-melodic-robot-state-publisher  ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```

### 创建工作空间：

### 下载：
```cd {ROS_WORKSPACE}/src```

```git clone https://github.com/TINY-KE/Barrett-7dof-WAM-arm-gazebo-moveit.git```

### 编译
```
cd {ROS_WORKSPACE}
```
```
catkin_make
```


## 二、运行Run
### 添加工作空间的路径
```source devel/setup.bash ```

### 在gazebo中可视化机器人urdf模型
+ 启动一个空白的gazebo环境
```roslaunch    barrett_model  create_world.launch  ```
+ 往gazebo中添加机器人，注意修改view_urdf.launch文件中的机器人模型
```roslaunch    barrett_model view_urdf.launch  ```


### 启动仿真环境（末端为相机）
+ 对应机器人模型 debug2_wam_7dof_wam_bhand.urdf.xacro
```roslaunch    wam_arm_moveit    wam_kinectv1_bringup_moveit_onlyrobot.launch ```




### 待实现： 启动仿真环境（末端为抓手）
+ 对应机器人模型 debug3_wam_7dof_wam_bhand.urdf.xacro
```roslaunch    wam_arm_moveit    wam_hand_bringup_moveit_onlyrobot.launch ```

