# iav_depthimage_to_laserscan

ROS node that convert a depth image to a laser scan for use with navigation and localization. Furthermore, it allows the use for one camera or more
(launch example for two cameras Asus Xtion Pro Live) and generates only one general laser scan for all of them, i.e. it allows create a 360° laser scan 
with using up to two cameras. For this just add one subscriber (with callback) and one variable Param (the parameters of the new camera) for each camera. 

It was implemented a Resolution Mapping that allows a bigger or smaller number of laser points than the resolution of the cameras.

The generated laser works like a 3D laser since shows not only what there are directly in front of the camera but what there are below and above as well, 
ensuring a better environmental vision. This is done through the parameters height_max (> 0) and height_min (< 0). 

How to build iav_depthimage_to_laserscan ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build iav_depthimage_to_laserscan node

How to run iav_depthimage_to_laserscan ros package
=====================================================================
    roslaunch iav_depthimage_to_laserscan itl_test.launch

You should see the scan result in the rviz.

NOTE: Remember, the launch was prepared for using with two cameras.

Transformation from Laser frame to camera_frame
=====================================================================
The transformations from laser to cameras were calculated from parameters and transmited via tf broadcaster to ROS system. So rviz can display the camera image in a correct way.
