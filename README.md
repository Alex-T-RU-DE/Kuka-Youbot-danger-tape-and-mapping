(project is not yet finished)

# Kuka-Youbot-danger-tape-recognition-and-mapping

The goal of this project is to add a danger tapes coorinates into occupancy grid with ROS and without depth camera. 


Software:
- ROS melodic
- Opencv
- Python


# Mapping

The [map](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) in the ROS is represented as a 1D array that contains three numbers: 
- -1 
-	0
- 100

The number ‘-1’ means that the area is not explored before.<br/>
The number ‘0’ means that area is clear and there is no obstacle.<br/> 
The number ‘100’ means that the area is occupied and there is an obstacle.<br/> 
The main problem of the map without any preparations is that it is too hard to track the world coordinates (like X and Y) in a 1D array. Moreover, the map’s size may vary asymmetrically depending on the robot’s path.

Before starting working with and changing the map's information during the run, we must modify the map parameters and make them big enough to keep the map’s borders unchanged to prevent the map’s extension. In our example, the map has the size 30x30 m (1472 pixels on X coordinate and square of 1472 pixels for Y with the resolution of 0.02m). 

After we prepared parameters for the map, we must modify our occupancy grid data according to the robot’s position and location of the danger tape relative to the robot. To do this, we are using transformation and rotation matrixes from the robot’s odometry to the camera’s position. Then, we are finding the distance from our camera to the tape’s points, that are lying on the ground, and defining its world coordinates. 

Once we defined its world coordinate, we must change the value to ‘100’ at some point in the [OccupancyGrid.data[]](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html). Since the Occupancy_grid message representing the map as a 1D array, we must find the right index, where the value has to be changed. The next pictures explain how to find the right index of [OccupancyGrid.data[]] for particular X and Y coordinate:

<blockquote class="imgur-embed-pub" lang="en" data-id="a/p97uP8M" data-context="false" ><a href="//imgur.com/a/p97uP8M"></a></blockquote><script async src="//s.imgur.com/min/embed.js" charset="utf-8"></script>



To start the program you can with following commands:

`rosrun line_recognition python tf_map.py`

`rosrun line_recognition cam.py`

`rosrun line_recognition map.py`

You can start these programs on the different machines within one ros network. (For instance you can run image processing on the nvidia jetson but your map will be on other remote machine)

Example: https://www.youtube.com/watch?v=Qk4FWbUZ-5A


