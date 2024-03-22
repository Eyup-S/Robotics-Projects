# PPP Robot
### Assignment:
1. Design a PPP robot using URDF.
2. Spawn a cube in x ∈ [0.5, 1], y ∈ [0.5, 1], z = 0.05 and push it in a random direction.
 

### How to Run

1. Put project_1 file to your ROS workspace
2. Open a terminal on the workspace folder and type "catkin_make" 
3. Then, type "roslaunch project_1 robot_gazebo.launch" to terminal
4. Open another termial and type "rosrun project_1 PPP_Controller"

### Notes
- If you want to re-run the program, first close gazebo and kill the terminal in which roslaunch runs. Then apply the steps above from the third.
