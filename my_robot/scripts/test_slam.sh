#!/bin/sh

# Launch home world and debloy my_robot in it
xterm  -e  " source devel/setup.bash; roslaunch my_robot world.launch " &
sleep 5

# Launch gmapping with tuned parameters to create a 2D occupancy grid map
xterm  -e  " source devel/setup.bash; roslaunch my_robot gmapping_demo.launch " &
sleep 5

# Launch rviz with saved configurations 
xterm  -e " source devel/setup.bash; roslaunch my_robot view_navigation.launch" &
sleep 5

# Launch teleop to teleoperate the robot and create a map 
xterm  -e  " source devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py "