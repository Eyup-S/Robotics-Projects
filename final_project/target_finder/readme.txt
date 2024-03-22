Put this package under src folder of your workspace. Then, build the workspace. Source the setup.bash file under Devel folder of the workspace.

This package requires Opencv. The main node uses Kinect input to find the position of a target/obstacle with a specified color. There is also another node that can be used to visualize or fine-tune the color limits.

Run detect_color node by running:
roslaunch target_finder detect_color.launch

3 windows will appear on the screen. One of them is the original image, one of them is the thresholded image and the other
one consists of the trackbars of H, S and V. These are hue, saturation and value. By adjusting
these values, best thresholded image for the target/obstacle can be found. The user should tweak H, S, V limits to obtain the desired thresholding.


Run target finder node by running:
roslaunch target_finder target_detect.launch

Enter the desired H, S ,V limits from the previous operation on color thresholding to this .launch file and save it.
This launch command will use the entered H,S,V limits
to find a target/obstacle with specified color and size, and by using depth input, it roughly
generates the x,y position of this target/obstacle in robocentric frame. Use these positions for navigation planning.

Note that the topic names in your simulations and the real robot tests may differ. Camera topics can be directly entered to the .launch files.
