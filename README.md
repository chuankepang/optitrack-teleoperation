# optitrack-teleoperation
UR10e teleoperation based on OptiTrack

# 1. 打开natnet接收motive的骨架数据
roslaunch natnet_ros_cpp natnet

# 2. 打开仿真的机械臂
roslaunch ur_e_gazebo ur10e.launch

# 3. 选择笛卡尔运动控制
rosrun rqt_controller_manager rqt_controller_manager
同时开启my_cartesian_motion_controller

# 4. 通过速度指令来控制笛卡尔运动
roslaunch cartesian_controller_utilities spacenav_to_pose.launch

# 5. 开启坐标变换
roslaunch arm_move start_server.launch 

# 6. 开启机械臂repeat
rosrun arm_move test_move.py

# 7. 可以直接复现motive传输的数据/播放存储的历史数据
播放motive/rosbag play xxxxx.bag
