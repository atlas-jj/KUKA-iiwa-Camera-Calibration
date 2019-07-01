# KUKA-iiwa-Camera-Calibration
Camera (Kinect Cam) Calibration routine for a KUKA iiwa Robot. 

This module was designed for our invited [live demo](https://webdocs.cs.ualberta.ca/~vis/IROS2017_demo.html "live demo") in IROS 2017, Vancouver, Canada.

# Prerequisite
+ You should have a KUKA iiwa Robot. 
  + **If you have other robots, you can change codes in the robot side, since UDP communication is universal.**
+ We are using AR Markers, the ROS package: **ar_track_alvar**, http://wiki.ros.org/ar_track_alvar
+ **No need to worry about how to make your KUKA iiwa robot working with ROS**. We make it possible.
  + If you plan to use KUKA iiwa Robot for a more complex and long duration task, you may need a standalone wrapper: our [**ROS_UDP_Wrapper**](https://github.com/atlas-jj/ROS-UDP-wrapper "ROS_UDP_Wrapper").
  + But for this **simple application**, we just integrate UDP Comm inside.
+ Control points are stored in result.txt. Read the points and use any transformation solver to solve.
+ That's it.

# How to use the code
+ Assume you are familiar with KUKA Sunrise JAVA Dev.
+ Assume you are familiar with ROS.
+ Assume you have a basic knowledge of using UDP communication.
+ You should know immediately how to use the codes.
+ Enjoy :)

# Auto-calibration
+ Yes, **we do support auto-calibration**.
  + previously, we did calibration in 20 minutes.
  + Now, it's within 5 minutes.
  + And we like it VERY MUCH, especially when we are in a live demo setup.

# Transformation Matrix Solver
+ Assume you know what's **SE(3)**.
+ We save correspondent coordinates in "result.txt".
  + e.g., [x,y,z] in camera's frame; [x,y,z] in robot's frame.
+ We do have a python script that can automatically read the txt file and calculate the transformation matrix. But I can't find it. It's a python script, I'm sure you can find it elsewhere.
+ As a result, I recommend [**My other solver**](https://github.com/atlas-jj/teleoperation-visual-servoing-SLAM/blob/master/wam_orbslam_solver/src/solver.cpp "My other solver").
+ I plan to incorporate this C++ solver into this repository in the future when I have a particular project.
