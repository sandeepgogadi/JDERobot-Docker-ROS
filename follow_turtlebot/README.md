[back to Robotics Playground](https://github.com/sandeepgogadi/Robotics-Playground)

[back to JDERobot ROS Docker](https://github.com/sandeepgogadi/JDERobot-Docker-ROS)

# Drone follow the ground robot

### Execute
1. In a terminal launch the gazebo simulator:
`gazebo ardrone-turtlebot.world`
2. In other terminal launch the turtlebot robot:
`kobukiViewer turtlebot.yml`
3. In another terminal lauch the follow_turtlebot component:
`python2 ./follow_turtlebot.py follow_turtlebot_conf.yml`

### Summary
The first step is to create a pid controller to use it in x and y axis. Next we use the input image from the drone to find the turtlebot. The top of the turtlebot is a green rectangle so using color filtering we can identify the turtlebot.If the green rectangle of the turtlebot is detected in the visual field then we will have to execute an algorithm to correct the position of the drone and get closer to the turtlebot. If it is not detected it is because it is not in the visual field of the drone and then you have to raise the drone until you get to see the turtlebot. One we have the turtlebot in our view we then calculate the error of the drone position. To do this our error will be the center of the rectangle detected by the turtlebot on each axis minus the center of the image of the drone camera on each axis. Using the pid controller we control the position in both directions so that position of turtlebot is centred in the image.

### Demo

![alt text](https://github.com/sandeepgogadi/JDERobot-Docker-ROS/blob/master/ffollow_turtlebot/follow_turtlebot.gif "Follow Turtlebot")
