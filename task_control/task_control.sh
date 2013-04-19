roslaunch task_control cart_controllers.launch &
rosparam set "task_control/testing" true
rosparam set "task_control/safety" 100.0
rosparam set "task_control/mode" simulation
rosparam set "task_control/enforce" true
rosrun task_control pr2_pose_trajectory_controller.py
