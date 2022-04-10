## Line Follower Assignment

### Summary ###
The package contains code that allows the robot to follow the yello line back and forth infinitely. The robot's angular velocity is adjusted using PID based on the location of the yellow line's centroid relative to the center of the camera. The gaol is to rotate the robot to keep the centroid of the yellow line as close as possible to the center of the camera image as possible. This way the robot can be as close to the yello line as possible as it moves. 

### How to run the code ###
A launch file has been created in the launch folder located in the package (/line_follower/launch/linemission.launch). It runs a simulated gazebo world with a basic line (it essentially calls the launch file linemission.launch in the prrexamples package) and the line_follower node that controls the robot given the image published to the "camera/rgb/image_raw/compressed" topic. In summary, run this bash command to launch the program:

```bash
roslaunch line_follower linemission.launch model:=waffle
```

### Code structure ###
In the line_follower.py file, a Follower class was created to store the states of the node necessary to properly control the robot. It also contains methods that calculate the centroid of the yellow line, and calculate the necessary angular velocity to move the robot as closed as the line as possible. The yello line is detected by making use the opencv mask's functionality. The program try to filter out all pixel within the yellow range, in which the location of the yello line's edge pixels can be found. It then makes use of the opencv moment method to calculate the centroid of the yllow line, and compare the x-coordinate of the centroid to the centroid of the camera image. PID value is calculated to steer the robot toward the centroid of the image. 

In order to allow the robot to double back along the line, the robot will try to rotate when there is no line visible ahead, until it find the line, it will move toward it. This way, the robot will know how to make u turn and follow the line again. This also allows the robot to move toward the line when the line is not immediately visible.