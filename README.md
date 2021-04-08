(project is not yet finished)

# Kuka-Youbot-danger-tape-recognition-and-mapping

The goal of this project is to add a danger tapes coorinates into occupancy grid with ROS and without depth camera. 


Software:
- ROS melodic
- Opencv
- Python

#Mapping



To start the program you can with following commands:

`rosrun line_recognition python tf_map.py`

`rosrun line_recognition cam.py`

`rosrun line_recognition map.py`

You can start these programs on the different machines within one ros network. (For instance you can run image processing on the nvidia jetson but your map will be on other remote machine)

Example: https://www.youtube.com/watch?v=Qk4FWbUZ-5A


