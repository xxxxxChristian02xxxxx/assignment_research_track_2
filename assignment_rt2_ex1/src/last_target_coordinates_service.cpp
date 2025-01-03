#include "ros/ros.h"
#include "assignment_rt2_ex1/Service_target_coordinates.h" 
#include <stdlib.h>
#include <stdio.h>

bool show_last_target_cooridinates(assignment_rt2_ex1::Service_target_coordinates::Request &req,assignment_rt2_ex1::Service_target_coordinates::Response &res){
	ros::NodeHandle nh;
	// checking if the parameters are present
	if (nh.hasParam("/des_pos_y") && nh.hasParam("/des_pos_x") ){
		// retreivng the parameters and giving thema as response
		nh.getParam("/des_pos_x", res.target_pos_x) ;
		nh.getParam("/des_pos_y", res.target_pos_y) ;

		return true;
	} else {
      		return false;
   	}
}

 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "last_target_coordinates_sent_server"); 
 	ros::NodeHandle n;
 	// initializ the servcice and binding the function
 	ros::ServiceServer service = n.advertiseService("/last_target_coordinates_sent_server",show_last_target_cooridinates);
	ros::spin();
 	return 0;

 }
