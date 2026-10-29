
### Part 1

To change the ROS1 tortoisebot waypoint server code such that its tests fail, in ~/simulation_ws/src/tortoisebot_waypoints/scripts/tortoisebot_action_server.py, uncomment lines 70-72 and 126-143.
To get the test to succeed, just make sure these lines are commented. 

In order to test for failure, uncomment the aforementioned lines (keep the comments themselves commented!), restart the waypoint server (no need to restart the sim), by performing the following:

    source /opt/ros/noetic/setup.bash
    cd ~/simulation_ws && catkin_make && source devel/setup.bash
    rosrun tortoisebot_waypoints tortoisebot_action_server.py

Wait about 15 seconds for the server to fully load, then perform the following test: 

    source /opt/ros/noetic/setup.bash
    cd ~/simulation_ws && catkin_make && source devel/setup.bash
    rostest tortoisebot_waypoints waypoints_test.test --reuse-master

In order to test for success, comment the aforementioned lines (keep the comments themselves commented!), restart the waypoint server (no need to restart the sim), by performing the following:

    source /opt/ros/noetic/setup.bash
    cd ~/simulation_ws && catkin_make && source devel/setup.bash
    rosrun tortoisebot_waypoints tortoisebot_action_server.py

Wait about 15 seconds for the server to fully load, then perform the following test: 

    source /opt/ros/noetic/setup.bash
    cd ~/simulation_ws && catkin_make && source devel/setup.bash
    rostest tortoisebot_waypoints waypoints_test.test --reuse-master

