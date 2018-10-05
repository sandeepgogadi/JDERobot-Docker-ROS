[back to Robotics Playground](https://github.com/sandeepgogadi/Robotics-Playground)

[back to JDERobot ROS Docker](https://github.com/sandeepgogadi/JDERobot-Docker-ROS)

# Drone follow the road

### Execute
1. In a terminal launch the gazebo simulator:
`gazebo road_drone_textures.world`
2. In other terminal lauch the follow_road component:
`python2 ./follow_road.py follow_road_conf.yml`

### Summary
The goal of this practice is to implement the logic that allows a quadricopter to follow a road. In order to do this, we will have to establish a color filter to segment road lines, and then develop an algorithm to follow them until the end of the road.

### Demo

![alt text](https://github.com/sandeepgogadi/JDERobot-Docker-ROS/blob/master/follow_road/follow_road.gif "Follow Road")
