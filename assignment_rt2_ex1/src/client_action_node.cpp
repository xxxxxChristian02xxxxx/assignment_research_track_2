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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// creation of the custom message
assignment_rt2_ex1::M_vel_pos vel_pose;
ros::Publisher pub_pos_vel;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher publish_distance;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


assignment_2_2024::PlanningActionFeedback::ConstPtr fdk;
int dim = 3;
std::vector<nav_msgs::Odometry> target_array;
int num_elements =0;


// 
void messageCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    vel_pose.x = msg->pose.pose.position.x;
    vel_pose.y = msg->pose.pose.position.y;
    vel_pose.vel_x = msg->twist.twist.linear.x;
    vel_pose.vel_z = msg->twist.twist.angular.z;
    
}

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
void feedbackCallback(const assignment_2_2024::PlanningActionFeedback::ConstPtr& msg) {
    fdk = msg;
}
assignment_2_2024::PlanningGoal set_target(){
	   	ros::NodeHandle nh;

	 	// following the composition of the file.action
 		// goal definition: geometry_msgs/PoseStamped target_pose
 		// result definition: N/A
 		// feedback definition: geometry_msgs/Pose actual_pose, string stat
 		
 		// if the robot is not moving, the ser will be asked to set the target position 
 		float target_pos_x,target_pos_y, target_ore_z;

 		
 		// building the goal 
 		assignment_2_2024::PlanningGoal goal;
 		goal.target_pose.pose.position.x = target_pos_x;
 		goal.target_pose.pose.position.y = target_pos_y;


 		
        	return goal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
assignment_2_2024::PlanningGoal set_target2(nav_msgs::Odometry target_element){
	   	ros::NodeHandle nh;

	 	// following the composition of the file.action
 		// goal definition: geometry_msgs/PoseStamped target_pose
 		// result definition: N/A
 		// feedback definition: geometry_msgs/Pose actual_pose, string stat
 		
 		// if the robot is not moving, the ser will be asked to set the target position 
 		//float target_pos_x,target_pos_y, target_ore_z;

 		
 		// building the goal 
 		assignment_2_2024::PlanningGoal goal;
 		goal.target_pose.pose.position.x = target_element.pose.pose.position.x;
 		goal.target_pose.pose.position.y = target_element.pose.pose.position.x;


 		
        return goal;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void call_service(){
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<assignment_rt2_ex1::Service_target_coordinates>("Service_target_coordinates");
	assignment_rt2_ex1::Service_target_coordinates srv;
	ROS_INFO("The target coordinates are: %f %f", srv.response.target_pos_x, srv.response.target_pos_y);

}
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
	ros::Subscriber laser_sub = nh.subscribe("scan", 1000, laserCallback);

	float target_pos_x,target_pos_y, target_ore_z;

	//assignment_2_2024::PlanningGoal goal = set_target();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
	assignment_2_2024::PlanningGoal goal = set_target2(target_array[0]);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	client.sendGoal(goal);


 	while(ros::ok()){
 		pub_pos_vel.publish(vel_pose);
 		ros::spinOnce();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		std::cout << "do you want to call the service? (y/n)" << std::endl;
		char cont;
		std::cin >> cont;
		if(cont == 'y'){
			call_service();
		}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	 	if (fdk != nullptr) {

			// making the goal if it is not moving
		   	 if (fdk->feedback.stat == "Target reached!" ||fdk->feedback.stat == "Target cancelled!" ) {

				// if i have reached all the targets then i will clear the vector				
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
					goal = set_target2(target_array[num_elements]);
				}else{
					// otherwise keep on sending the target
					goal = set_target2(target_array[num_elements]);
				}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				//goal = set_target();
				client.sendGoal(goal);
				num_elements++;	

				
			} else if (fdk->feedback.stat == "State 0: go to point" || fdk->feedback.stat == "State 1: avoid obstacle") { // giving possibility to start again if moving
				
				decide_action_in_moving(client);
					//ros::spinOnce();
					//continue;	
			}
			fdk = nullptr;
		}
 	}
 	
	return 0;
}




