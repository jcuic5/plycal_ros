# plycal_ros
Lidar-camera calibration simply using plycal (https://github.com/ram-lab/plycal). This is just a ros package with snippets to prepare data for plycal, and also dynamically visualize the calibration result in rviz

### Change log:

- tested on Ubuntu 18.04

- No changes made on the core functionalities by plycal, except adding PCL package dependency for PlyCal_qt (in its CMakeLists.txt) to solve compiling error
- Added ROS node to subscribe, parse and save image and pointcloud data. If calibration result is already provided, it also visualize the projection in RVIZ