[back to Robotics Playground](https://github.com/sandeepgogadi/Robotics-Playground)

[back to JDERobot ROS Docker](https://github.com/sandeepgogadi/JDERobot-Docker-ROS)

# Color Filter

### Execution

1. Launch cameraserver component
`cameraserver cameraserver_conf.cfg`

2. Launch color filter component in other terminal
`python2 ./color_filter.py color_filter_conf.yml`

### Summary

The first step is to obtain the image provided by cameraserver (in this case it will be the one of the video).The images provided by the cameraserver have a lot of noise. For this reason it is advisable to use a smoothing to eliminate this noise. In the solution, a Gaussian filter has been used. The image is in RGB, so we do a color space conversion to HSV to be able to recognize the balls in an environment of varied luminosity. The next step is to apply a color filter. We apply the color filter with these minimum and maximum values ​​for tone, saturation and intensity. With this function we will obtain a thresholded image, where the desired objects will appear in white and the rest in black. The closing morphological operation is used to fill the holes that remain unfiltered in the balls trying to improve the thresholding obtained. We copy the original and filtered image so as not to modify them. OpenCV findcontour is applied to detect the contour of the filtered area. OpenCV approxPolyDP is used to approximate the contours. The boundingRect is used to detect the rectangle that fits the detected contour. Finally, a rectangle is painted around the areas detected as a ball, as well as a circumference.
