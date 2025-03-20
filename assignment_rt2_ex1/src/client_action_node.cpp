/**
 * \file client_action_node.cpp
 * \brief This is the client node that will send the target to the server and will receive the feedback from the server n order to move the robot
 * \author Christian Negri Ravera
 * \date 2021-07-01
 * \version 1.0
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *  /odom: [nav_msgs/Odometry] <BR>
 *  /reaching_goal/feedback: [assignment_2_2024/PlanningAction] <BR>
 * 	/scan: [sensor_msgs/LaserScan] <BR>
 * 
 * Publishes to: <BR>
 * /pos_vel_custom_mess: [assignment_rt2_ex1/M_vel_pos] <BR>
 * /distance_from_obstacle: [std_msgs/Float32] <BR>
 * 
 * Description: <BR>
 * The node provides the possibility to give a list of targets to the robot and make it reach them. 
 * Goals can be chnaged while the robot is moving.
 * 
 */

#include "ros/ros.h"
// librerie abbligatorie
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// librerie del file action contenuto nel pacchetto
#include <assignment_2_2024/PlanningAction.h>
// include of the message custom 
#include <assignment_rt2_ex1/M_vel_pos.h>
#include <nav_msgs/Odometry.h>
// useful libraries
#include <iostream>
#include <string>
#include <std_msgs/String.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <vector> 
#include "assignment_rt2_ex1/Service_target_coordinates.h"
 #include <std_msgs/Float32.h>
 #include <sensor_msgs/LaserScan.h>

// creation of the custom message
assignment_rt2_ex1::M_vel_pos vel_pose;
ros::Publisher pub_pos_vel;
ros::Publisher publish_distance;							///< publisher to publish the distance from the obstacles
assignment_2_2024::PlanningActionFeedback::ConstPtr fdk;	///< feedback from the server reference
int dim = 3; 												///< dimension of the vector containing the target coordinates
std::vector<nav_msgs::Odometry> target_array; 				///< vector containing the target coordinates
int num_elements =0; 										///< index to keep track the target selected


/**
 * \brief Callback function for the position of the robot 
 * \param msg [nav_msgs::Odometry::ConstPtr] the message containing the position of the robot
 * \return none 
 * 
 * Description: <BR>
 * This function is the callback for the position of the robot. It will store the position and the velocity of the robot in the global variable vel_pose
 */
void messageCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    vel_pose.x = msg->pose.pose.position.x;
    vel_pose.y = msg->pose.pose.position.y;
    vel_pose.vel_x = msg->twist.twist.linear.x;
    vel_pose.vel_z = msg->twist.twist.angular.z;
    
}
/**
 * \brief Callback function for the feedback of the server
 * \param msg [assignment_2_2024::PlanningActionFeedback::ConstPtr] the message containing the feedback of the server
 * \return none
 * 
 * Description: <BR>
 * This function is the callback for the feedback of the server. It will store the feedback in the global variable fdk
 */
void feedbackCallback(const assignment_2_2024::PlanningActionFeedback::ConstPtr& msg) {
    fdk = msg;
}

/**
 * \brief Callback function for the laser scan
 * \param msg [sensor_msgs::LaserScan::ConstPtr] the message containing the laser scan
 * \return none
 * 
 * Description: <BR>
 * This function is the callback for the laser scan. It will store the minimum distance from the obstacles in the global variable closest_obstacle
 */
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    float min_distance = std::numeric_limits<float>::infinity();
    for (const auto& range : msg->ranges) {
        if (range < min_distance) {
            min_distance = range;
        }
    }
    std_msgs::Float32 closest_obstacle_msg;
    closest_obstacle_msg.data = min_distance;
    publish_distance.publish(closest_obstacle_msg);
}

/**
 * \brief Function to decide the action while the robot is moving
 * \param client [actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>&] the client to send the action
 * \return none
 * 
 * Description: <BR>
 * This function will ask the user if he wants to cancel the target while the robot is moving. If the user wants to cancel the target, the function will send the cancelGoal to the client
 */
void decide_action_in_moving( actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>& client){
	 int user_block_decision=0;

	 std::cout<<"The robot is moving \n1 -- cancel the target "<< std::endl;	
	while (!(std::cin >> user_block_decision)) {  
        	std::cin.clear();  
       	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
       	std::cout<<"The robot is moving \n1 -- cancel the target "<< std::endl;	  
    	}
	 if (user_block_decision == 1){
	 	client.cancelGoal();
	 	ros::Duration(1).sleep();
	 } 		
}


/**
 * \brief Function to set the target
 * \param target_element [nav_msgs::Odometry] the target coordinates
 * \return assignment_2_2024::PlanningGoal the goal to reach the target
 * 
 * Description: <BR>
 * This function will set the target to reach
 */
assignment_2_2024::PlanningGoal set_target(nav_msgs::Odometry target_element){
	   	ros::NodeHandle nh; 		
 		// building the goal 
 		assignment_2_2024::PlanningGoal goal;
 		goal.target_pose.pose.position.x = target_element.pose.pose.position.x;
 		goal.target_pose.pose.position.y = target_element.pose.pose.position.x;


 		
        return goal;
}
/**
 * \brief Function to call the service
 * \param none
 * \return none
 * 
 * Description: <BR>
 * This function will call the service to get the target coordinates
 */
void call_service(){
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<assignment_rt2_ex1::Service_target_coordinates>("Service_target_coordinates");
	assignment_rt2_ex1::Service_target_coordinates srv;
	ROS_INFO("The target coordinates are: %f %f", srv.response.target_pos_x, srv.response.target_pos_y);

}
 
/**
 * \brief Main function
 * \param argc [int] number of arguments
 * \param argv [char**] arguments
 * \return 0 since it is main function
 * 
 * Description: <BR>
 * This is the main function of the node. It will initialize the node, create the action client, and wait for the server to start.
 * Initialize the publisher and the subscriber.
 * Ask the user for the target coordinates the robot must reach.
 * Gives the possibility to eliminate the target while the robot is moving.
 * After all goals reached, the user can insert new targets.
 */
int main (int argc, char** argv){
	// Inizializza ROS
    	ros::init(argc, argv, "client_action_target");


   	// Crea il NodeHandle dopo ros::init()
   	ros::NodeHandle nh;
	// creation of the instance of the action service 
	actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> client("/reaching_goal", true);


	// calling the server and waitng for it starts
	client.waitForServer();
	
	// Publisher to publish the velocity and position
	pub_pos_vel = nh.advertise<assignment_rt2_ex1::M_vel_pos>("pos_vel_custom_mess",100); 
	publish_distance = nh.advertise<std_msgs::Float32>("distance_from_obstacle", 1000);
	// subscriber on odom
	ros::Subscriber sub = nh.subscribe("odom", 1000, messageCallback);
	ros::Subscriber feedback_sub = nh.subscribe("/reaching_goal/feedback", 10, feedbackCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 1000, laserCallback);

	float target_pos_x,target_pos_y, target_ore_z;


	int j =0;
	while(true){
		std::cout << "Enter the target coordinates:" << std::endl; 
		std::cout << "X:"; std::cin >> target_pos_x;
		std::cout << "Y:"; std::cin >> target_pos_y;
		target_array[j].pose.pose.position.x =target_pos_x ;
		target_array[j].pose.pose.position.y =target_pos_y;
		std::cout << "Do you want to add another target? (y/n)" << std::endl;
		char cont;
		std::cin >> cont;
		if(cont == 'n'){
			break;
		} 
		j++;
	}
	assignment_2_2024::PlanningGoal goal = set_target(target_array[0]);
	client.sendGoal(goal);


 	while(ros::ok()){
 		pub_pos_vel.publish(vel_pose);
 		ros::spinOnce();
	
		std::cout << "do you want to call the service? (y/n)" << std::endl;
		char cont;
		std::cin >> cont;
		if(cont == 'y'){
			call_service();
		}
	


	 	if (fdk != nullptr) {

			// making the goal if it is not moving
		   	 if (fdk->feedback.stat == "Target reached!" ||fdk->feedback.stat == "Target cancelled!" ) {

				// if i have reached all the targets then i will clear the vector				
	
				if(num_elements == target_array.size()){
					num_elements =0;	
					target_array.clear();			
				}
				// if vector is empty then fill it in
				if(target_array.empty()){
					char cont;
					int i =0;
					while(true){
						std::cout << "Enter the target coordinates:" << std::endl; 
						std::cout << "X:"; std::cin >> target_pos_x;
						std::cout << "Y:"; std::cin >> target_pos_y;
						target_array[i].pose.pose.position.x =target_pos_x ;
						target_array[i].pose.pose.position.y =target_pos_y;
						std::cout << "Do you want to add another target? (y/n)" << std::endl;
						std::cin >> cont;
						if(cont == 'n'){
							break;
						} 
						i++;
					}
					goal = set_target(target_array[num_elements]);
				}else{
					// otherwise keep on sending the target
					goal = set_target(target_array[num_elements]);
				}
	

				//goal = set_target();
				client.sendGoal(goal);
				num_elements++;	

				
			} else if (fdk->feedback.stat == "State 0: go to point" || fdk->feedback.stat == "State 1: avoid obstacle") { // giving possibility to start again if moving
				
				decide_action_in_moving(client);
				}
			fdk = nullptr;
		}
 	}
 	
	return 0;
}




