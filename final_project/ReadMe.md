# Following a Colored Object Using Turtlebot
### Assignment:
Follow blue object detected by the camera on Turtlebot.


### TODO
- Implementing of Dynamic Window Approach is not working properly in dwa_planner.cpp. Some gain adjustments may be made.

### How to Run
- Put these packages under src folder of your workspace. 
- Then, build the workspace. 
- Source the setup.bash file under Devel folder of the workspace.
- type: roslaunch turtlebot spawn_robot.launch
- In another terminal type: roslaunch target_finder target_detect.launch
- Lastly run this command: rosrun follower runfollower


