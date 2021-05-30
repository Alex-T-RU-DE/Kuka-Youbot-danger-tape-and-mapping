#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/String.h"
#include <math.h>
#include <iostream>

using namespace std;

class map_summ
{
 protected:
	ros::Publisher map_pub;
	ros::NodeHandle n;
	nav_msgs::OccupancyGrid::ConstPtr gmapping_map;
	nav_msgs::OccupancyGrid tape_map;
	nav_msgs::OccupancyGrid sum_map;
	ros::Subscriber gmapping_map_sub;
	ros::Subscriber tape_map_sub;

		
 public:
   map_summ()
	{   
	    map_pub = n.advertise<nav_msgs::OccupancyGrid>("combined_map", 1000);
	    //getting once the parameters of the main map build by gmappind and 2dslam
	    gmapping_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", n);
	    sum_map.info = gmapping_map -> info;
	    sum_map.data = gmapping_map -> data;
	    sum_map.info.map_load_time = ros::Time::now();
	    sum_map.header.frame_id = "combined_map";
	    tape_map_sub = n.subscribe("new_map", 1000, &map_summ::update_t_map, this);
	   
	}
   	
  void update_t_map(const nav_msgs::OccupancyGrid msg)
	{   int arr_size = static_cast<int>(sum_map.info.width * sum_map.info.height);
		if(arr_size == static_cast<int>(msg.info.width * msg.info.height))
		 { 
		  for (int i = 0; i < arr_size; i++)
			if(static_cast<int>(msg.data[i]) == 100)
				sum_map.data[i]=100;
		  map_pub.publish(sum_map);
		 }
		else
			cout<< "Error: sizes are different (" << arr_size << " and " <<
				 static_cast<int>(msg.info.width) * static_cast<int>(msg.info.height) << ")" << endl;
		
	}
    
};


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "summ_node");
  map_summ var;
  while(ros::ok())
  	ros::spin();
  
  return 0;
}
