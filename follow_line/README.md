[back to Robotics Playground](https://github.com/sandeepgogadi/Robotics-Playground)

[back to JDERobot ROS Docker](https://github.com/sandeepgogadi/JDERobot-Docker-ROS)

# Visual follow-line behavior on a Formula1 car

### Execute
1. Launch gazebo world:
`roslaunch /opt/jderobot/share/jderobot/launch/f1.launch`
2. launch the code:
`python2 ./follow_line.py follow_line_conf.yml`

### Summary

First step is to convert the image from the camera which is in RGB colorspace to HSV colorspace. We are only interested in the part of the image containing the road so we crop the image containing the road. The line is red in color, using the upper and lower bounds in the hsv color space we create a binary mask containing the road line. Then we calculate the position x of center of the line horizontally. Using this we can compute the error which is difference between image width and x.

To control the speed of the car we use a PID controller tuned with twiddle algorithm. The PID gain parameters are 0.2, 0.004, 3.0.   
