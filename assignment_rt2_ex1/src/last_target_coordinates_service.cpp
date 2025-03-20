/**
 * \file last_target_coordinates_service.cpp
 * \brief This is the server node that will provide the last target coordinates sent by the client
 * \author Christian Negri Ravera
 * \date 2021-07-01
 * \version 1.0
 * 
 * \details
 * Subscribes to: <BR>
 *  none <BR>
 * 
 * 
 * Publishes to: <BR>
 * none <BR>
 * Description: <BR>
 * The node provides the possibility to show the last target coordinates sent by the client
 */

#include "ros/ros.h"
#include "assignment_rt2_ex1/Service_target_coordinates.h" 
#include <stdlib.h>
#include <stdio.h>

/**
 * \brief Function to show the last target coordinates
 * \param req [assignment_rt2_ex1::Service_target_coordinates::Request] the request to the server
 * \param res [assignment_rt2_ex1::Service_target_coordinates::Response] the response from the server
 * \return bool true if the parameters are present, false otherwise
 * 
 * Description: <BR>
 * This function will show the last target coordinates sent by the client
 */
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

/**
 * \brief Main function
 * \param argc [int] number of arguments
 * \param argv [char**] arguments
 * \return 0 since it is main function
 * 
 * Description: <BR>
 * This is the main function of the node. It will initialize the node, create the service and wait for the client to call it.
 * 
 */
 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "last_target_coordinates_sent_server"); 
 	ros::NodeHandle n;
 	// initializ the servcice and binding the function
 	ros::ServiceServer service = n.advertiseService("/last_target_coordinates_sent_server",show_last_target_cooridinates);
	ros::spin();
 	return 0;

 }
