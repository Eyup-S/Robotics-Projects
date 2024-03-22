# Forward and Inverse Kinematics
### Assignment
1. Move robot through all of its workspace 
2. spawn N spheres in coordinate space and touch them one by one

### How to Run

put rrrbot folder in your workspace and run the following command
> catkin_make

Then run the following command to launch gazebo
> roslaunch rrrbot spawn_robot.launch

In a new terminal run this command
> roslaunch rrrbot run_controllers.launch

Lastly, run show_workspace or touch_spheres nodes.
> rosrun rrrbot show_workspace
or
> rosrun rrrbot touch_spheres


**Note:** touch_spheres node is not working as expected.
