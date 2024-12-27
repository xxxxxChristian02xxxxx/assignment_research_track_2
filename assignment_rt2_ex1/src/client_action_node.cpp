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
ros::Publisher pub_pos_vel;
int moving_toggle = false;
assignment_2_2024::PlanningActionFeedback::ConstPtr fdk;

void messageCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    vel_pose.x = msg->pose.pose.position.x;
    vel_pose.y = msg->pose.pose.position.y;
    vel_pose.vel_x = msg->twist.twist.linear.x;
    vel_pose.vel_z = msg->twist.twist.angular.z;
    
}

void decide_action_in_moving( actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>& client){
	 int user_block_decision=0;

	 std::cout<<"The robot is moving \n1 -- cancel the taget "<< std::endl;	
	 std::cin >> user_block_decision;
	 if (user_block_decision == 1){
	 	client.cancelGoal();
	 	ros::Duration(1).sleep();
	 }

	 
 		
 		
}


assignment_2_2024::PlanningGoal set_target(){
	   	ros::NodeHandle nh;

	 	// following the composition of the file.action
 		// goal definition: geometry_msgs/PoseStamped target_pose
 		// result definition: N/A
 		// feedback definition: geometry_msgs/Pose actual_pose, string stat
 		
 		// if the robot is not moving, the ser will be asked to set the target position 
 		float target_pos_x,target_pos_y, target_ore_z;
 		std::cout << "Enter the target coordinates:" << std::endl; 
 		std::cout << "X:"; std::cin >> target_pos_x;
  		std::cout << "Y:"; std::cin >> target_pos_y;
  		std::cout << "Orientation:"; std::cin >> target_ore_z;
 		
 		// building the goal 
 		assignment_2_2024::PlanningGoal goal;
 		goal.target_pose.pose.position.x = target_pos_x;
 		goal.target_pose.pose.position.y = target_pos_y;
 		goal.target_pose.pose.orientation.z = target_ore_z;
 		
		// memorizing the parameters int he server 
       	nh.setParam("/last_target/x", target_pos_x);
        	nh.setParam("/last_target/y", target_pos_y);
        	nh.setParam("/last_target/orientation", target_ore_z);
        	return goal;
}
void feedbackCallback(const assignment_2_2024::PlanningActionFeedback::ConstPtr& msg) {
    fdk = msg;
    //ROS_INFO("Feedback ricevuto: %s", fdk->feedback.stat.c_str());
 
}

 
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

	// subscriber on odom
	ros::Subscriber sub = nh.subscribe("odom", 1000, messageCallback);
	ros::Subscriber feedback_sub = nh.subscribe("/reaching_goal/feedback", 10, feedbackCallback);


	assignment_2_2024::PlanningGoal goal = set_target();
        client.sendGoal(goal);
 	while(ros::ok()){
 		pub_pos_vel.publish(vel_pose);
 		ros::spinOnce();

	 	if (fdk != nullptr) {
		   	 ROS_INFO("Feedback ricevuto: %s", fdk->feedback.stat.c_str());
		   	 if (fdk->feedback.stat == "Target reached!" ||fdk->feedback.stat == "Target cancelled!" ) {
				goal = set_target();
				client.sendGoal(goal);	

				
			} else if (fdk->feedback.stat == "State 0: go to point" || fdk->feedback.stat == "State 1: avoid obstacle") {
				decide_action_in_moving(client);
				//if (moving_toggle){
					ros::spinOnce();
				//	moving_toggle = false;
					continue;
				//}
			}
			fdk = nullptr;
		}
 	}
 	
	return 0;
}




