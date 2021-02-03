# Kuka-Youbot-danger-tape-and-mapping
Project is still in progress. Now we are replacing usual webcam by depth cam.


The goal of this project is to add a danger tapes coorinates into occupancy grid with ROS. 

Software:
-ROS melodic
-Opencv
-Python

in order to start program you could use following command:

`roslaunch line_recognition line_detector.launch`

or you can simply start in from the source folder with following commands:

`python tf_map.py`

`python cam.py`

`python map.py`

You can start these codes on the different machines within one ros network. (For instance you can run image processing on the nvidia jetson but your map will be on other remote machine)

Example without depth camera: https://youtu.be/rVSTNN2OTR8


