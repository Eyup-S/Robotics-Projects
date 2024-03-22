# Navigating Mobile Robot using Artificial Potential Field Method
### Assignment:
Use the APF(artificial potential field) approach to navigate your robot from the initial position to an arbitrary goal position in an environment with obstacles. Your program should read the task from a file as follows:
gx gy
o1x o1y ρ1 h1
..
.
oMx oNy ρM hM

### How to Run
- Put this package under src folder of your workspace. 
- Then, build the workspace. 
- Source the setup.bash file under Devel folder of the workspace.
- type: roslaunch turtlebot spawn_robot.launch
- For the task 1.1 use following command: rosrun turtlebot calculate_position   linear.x   angular.z
- For the task 1.2 first set goal position and obstacle positions using /config/robot_task.txt
- Then use following command: rosrun turtlebot apf_planning
- Robot path and obstacles can be observed running python /src/turtlebot/config/plot.py (plot of obstacles can be changed on plot.py)  
