#include "ros/ros.h"
#include "assignment_rt2_ex1/Service_target_coordinates.h" 
#include <stdlib.h>
#include <stdio.h>

bool show_last_target_cooridinates(assignment_rt2_ex1::Service_target_coordinates::Request &req,assignment_rt2_ex1::Service_target_coordinates::Response &res){
	ros::NodeHandle nh;
	// checking if the parameters are present
	if (nh.hasParam("/last_target/x") && nh.hasParam("/last_target/y") && nh.hasParam("/last_target/orientation")){
		// retreivng the parameters and giving thema as response
		nh.getParam("/last_target/x", res.target_pos_x) ;
		nh.getParam("/last_target/y", res.target_pos_y) ;
		nh.getParam("/last_target/orientation", res.target_orie_z);
		return true;
	} else {
      		return false;
   	}
}

 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "last_target_coordinates_sent_server"); 
 	ros::NodeHandle n;
 	ros::ServiceServer service = n.advertiseService("/last_target_coordinates_sent_server",show_last_target_cooridinates);
	ros::spin();
 	return 0;

 }
