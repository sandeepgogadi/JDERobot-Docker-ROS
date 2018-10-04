[back to]()
[back to]()

# Color Filter

In this project, the objective is to segment some objects either a ball or several balls.

## Execution

[Install JDERobot environment](http://jderobot.org/Installation)

Launch cameraserver component
`cameraserver cameraserver_conf.cfg`

Launch color filter component in other terminal
`python2 ./color_filter.py color_filter_conf.yml`

## Summary

1. The first step is to obtain the image provided by cameraserver.
2. The images provided by the cameraserver have a lot of noise. For this reason it is advisable to use a smoothing to eliminate this noise. In the solution, a Gaussian filter has been used.
3. The next step is to apply a color filter. We apply the color filter with minimum and maximum values ​​for red, green, blue. With this function we will obtain a thresholded image, where the desired objects will appear in white and the rest in black.

## Result

![alt text]( 'Red ball')

![alt text]( 'Red & Blue ball')
