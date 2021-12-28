# plycal_ros
Lidar-camera calibration simply using plycal (https://github.com/ram-lab/plycal). This is just a ros package with snippets to prepare data for plycal, and also dynamically visualize the calibration result in rviz

## Change log:

- tested on Ubuntu 18.04
- No changes made on the core functionalities by plycal, except adding PCL package dependency for PlyCal_qt (in its CMakeLists.txt) to solve compiling error. 
- Added English version of the instructions (plycal/doc/README_en.md)
- Added ROS node to subscribe, parse and save image and pointcloud data. If calibration result is already provided, it also visualize the projection in RVIZ

## Usage

```bash
# Put the package in your workspace
cd {YOUR_WORKSPACE}
caktin build plycal_ros
```

Then configure topics to be compatible with your calibration rosbag files and make sure of the message types in *bag_sync.py*

```
roslaunch plycal_ros data_preprocess.launch
rosbag play {YOUR_PATH_TO_BAGFILE}
```

You could run with rviz to see the what's the currently frame like, and in the console where you launched the process, press ENTER to save the current frame (pair of image and point cloud). They will be saved in *plycal/data/image_orig* and *plycal/data/pointcloud_bin*

Now we have the data for plycal to perform the calibration. Before that, we need to convert the point cloud file format from .bin to .pcd. To do that, use *scripts/process_pcd.py*

Check the prepared data in the two data folders *plycal/data/image_orig* and *plycal/data/pointcloud*. And now we could simply make use of plycal to do the calibration. Refer to *plycal/README.md*