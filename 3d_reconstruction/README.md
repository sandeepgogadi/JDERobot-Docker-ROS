[back to Robotics Playground](https://github.com/sandeepgogadi/Robotics-Playground)

[back to JDERobot ROS Docker](https://github.com/sandeepgogadi/JDERobot-Docker-ROS)

# Visual 3D reconstruction from a stereo pair of RGB cameras

### Execute

1. Setup 3DViz
`bash setup.sh`
2. Launch the world:
`gazebo reconstruccion3D.world`
3. Then, run the 3d_reconstruction component:
`python2 3d_reconstruction.py 3d_reconstruction_conf.yml`
4. Finally, launch 3D viewer tool:
`cd 3DVizWeb/ && npm start`

### Summary

The first phase is to be able to identify where one interest point in one camera is in the other camera of the stereo pair. For that task, we first use a Canny filter over the image of one of the cameras in order to extract the borders that appear in our image. Each pixel of those extracted borders will be considered as an interest point that we will need to look for in the other image. We are focusing on reconstruction border points as they have enough texture to be located in a decent way in the corresponding image. The absence of texture is the biggest issue that stereo vision has to face.

Once we have that point to analyze, we project it in the 3D space using the graficas2opticas method followed by backproject. These methods firstly put into the optical coordinates our graphical interest point and then backproject it towards the space. With the 3D point the backproject method  the 3D position of where our first camera is, we then project those 3D points into the second camera using the project method. After passing those two points through the opticas2graficas method, we obtain two points in the second camera that form a line known as the epipolar. This epipolar line passes through the interest point we found in the first camera but in the second one, allowing us to narrow the search box.

The first step we take with the epipolar line is to confirm if that line passes through the image. If not we discard that interest point and we proceed to the following one. But if the line satisfies this condition, we then proceed to identify where that point lays in the second image using Template Matching along the epipolar line. As we know that the scene that we are trying to reconstruct does not contains objects that are too close to the cameras, we restrict the search area along the epipolar line.

By taking these steps, we are able to know where one interest point in one camera is in the other one. This leads us to the second phase of the assignment, which is to determine where in the space that interest point is located.

The second phase consists of intersecting the lines that form each of those same interest points in each image and the focal point of each camera, which is a known point that is given with the calibration XML. With these two pairs of points, two lines we will be defined and intersected, obtaining the 3D position of the interest point that we located in one camera at first.

In order to know the two pairs of 3D points, we use again the graficas2opticas method followed by backproject to obtain the 3D points that are along the line that form the focal point and the interest point with each camera. With those 3D point and the ones of the cameras focal point, we define the direction vector of each line. We then obtain the perpendicular vector to those to lines by applying the cross product of the lines. The intersection of the plane that forms one line with the perpendicular vector and the other line, will give us the 3D point where both lines intersect.

This concept would be correct if it wasn't for the error that is introduced in the calibration of the cameras. For that reason we have to repeat that process with the two lines as those lines do not intersect in the 3D space due to the calibration error. Once we have those two 3D points, the center point of the line that form will be considered as the 3D point that we were seeking for.

Repeating this whole process for each interest point that we find in the Canny image of one of the cameras, we will achieve a 3D reconstruction of the world we are working with.
