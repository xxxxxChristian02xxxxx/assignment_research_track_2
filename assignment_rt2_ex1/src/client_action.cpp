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
 
 // creation of the custom message
assignment_rt2_ex1::M_vel_pos vel_pose;
 
 
void messageCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    vel_pose.x = msg->pose.pose.position.x;
    vel_pose.y = msg->pose.pose.position.y;
    vel_pose.vel_x = msg->twist.twist.linear.x;
    vel_pose.vel_z = msg->twist.twist.angular.z;
}
 
int main (int argc, char** argv){
	ros::init(argc, argv, "client_action_target");
	ros::NodeHandle nh;
	
	// Publisher to publish the velocity and position
	ros::Publisher pub_pos_vel = nh.advertise<assignment_rt2_ex1::M_vel_pos>("new_topic",10); // change name 

	// subscriber on odom
	ros::Subscriber sub = nh.subscribe("odom", 1000, messageCallback);
	
	// creation of the instance of the action service 
	actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> client("/reaching_goal",true);
	// calling the server and waitng for it starts
	client.waitForServer();

	// retrieving the feedback from the service 
	assignment_2_2024::PlanningActionFeedback::ConstPtr fdk = ros::topic::waitForMessage<assignment_2_2024::PlanningActionFeedback>("/reaching_goal/feedback");
	
	 // listening to the feedback and seeing if the robot is moving, if yes, the user can choose if to stop it or to let it move
	int moving_toggle = true;
	if ((fdk->feedback.stat !="Target reached!")&& (fdk->feedback.stat != "Target cancelled")){
 			int user_block_decision=0;
 			std::cout<<"The robot is moving \n 1 -- cancel the taget \n 2 -- keep on moving "<< std::endl;	
 			std::cin >> user_block_decision;
 			
 			if (user_block_decision){
 				client.cancelGoal();
 				moving_toggle= false;
 			}
	}
	
	
	
	
 	while(ros::ok() && !moving_toggle){
 		
 		pub_pos_vel.publish(vel_pose);
 		
 		// following the composition of the file.action
 		// goal definition: geometry_msgs/PoseStamped target_pose
 		// result definition: N/A
 		// feedback definition: geometry_msgs/Pose actual_pose, string stat
 
 		// if the robot is not moving, the ser will be asked to set the target position 
 		double target_pos_x,target_pos_y, orientation;
 		std::cout << "Enter the target coordinates:" << std::endl; 
 		std::cout << "X:"; std::cin >> target_pos_x;
  		std::cout << "Y:"; std::cin >> target_pos_y;
  		std::cout << "Orientation:"; std::cin >> orientation;
 		
 		// building the goal 
 		assignment_2_2024::PlanningGoal goal;
 		goal.target_pose.pose.position.x = target_pos_x;
 		goal.target_pose.pose.position.y = target_pos_y;
 		goal.target_pose.pose.orientation.z = orientation;
 		
 		// sending the goal 
 		client.sendGoal(goal);
 		
 		ros::spinOnce();
 	
 	}
 	

	
	
	
	
	
	return 0;
}

