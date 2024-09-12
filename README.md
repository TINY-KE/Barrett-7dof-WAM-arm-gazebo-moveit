# Barrett-7dof-WAM-gazebo-moveit

# Build
'''
sudo apt install ros-melodic-joint-state-controller
mkdir ws_wam_arm  && cd ws_wam_arm && mkdir src &&  cd src
git clone https://github.com/TINY-KE/Barrett-7dof-WAM-arm-gazebo-moveit.git
cd ..
catkin_make
'''

# run
'''
source devel/setup.bash 
roslaunch    wam_arm_moveit    wam_kinectv1_bringup_moveit_onlyrobot.launch 
'''
