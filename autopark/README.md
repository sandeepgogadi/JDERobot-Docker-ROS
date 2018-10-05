[back to Robotics Playground](https://github.com/sandeepgogadi/Robotics-Playground)

[back to JDERobot ROS Docker](https://github.com/sandeepgogadi/JDERobot-Docker-ROS)

# Autoparking

### Execute

To launch the example, follow the steps below:
1. Run Gazebo simulator:
`gazebo autopark.world`
2. Running the practice and the user interface:
`python2 autopark.py autopark_conf.yml`

### Summary
The goal of this practice is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must find a parking space and park properly.

The algorithm uses front, rear and right laser scanners to check for space and parks the car in that location.
