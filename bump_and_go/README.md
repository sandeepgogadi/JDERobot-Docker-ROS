[back to Robotics Playground](https://github.com/sandeepgogadi/Robotics-Playground)

[back to JDERobot ROS Docker](https://github.com/sandeepgogadi/JDERobot-Docker-ROS)

# Vacuum Cleaner bump and go

### Execute

Once created the state machine and set the code for each state,
To launch the example you only have to follow the following steps:

1. Run Gazebo:
`roslaunch kobuki-simple-ros.launch`
2. Execution of the bum&go component:
`python bump_and_go.py bump_and_go.yml --displaygui=true`

### Summary

The intention of this excersise is to program a basic behaviour of bump-spin using a finite state machine. For that, we will use GUI tool, that allows you to create your own states machine in an intuitive way.

### Demo

![alt text](https://github.com/sandeepgogadi/JDERobot-Docker-ROS/blob/master/bump_and_go/bumpngo.gif "Bump and Go") 
