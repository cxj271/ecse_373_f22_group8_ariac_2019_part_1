#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ur_kinematics/ur_kin.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"



std_srvs::Trigger begin_comp;
//std_srvs::SetBool my_bool_var;
//my_bool_var.request.data = true;

bool first_run = true;
int service_call_succeeded;

std::vector<osrf_gear::Order> order_vec;

std::vector<osrf_gear::LogicalCameraImage> lc_bin_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_agv_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_fault_vec;

sensor_msgs::JointState joint_states;

trajectory_msgs::JointTrajectory desired;
trajectory_msgs::JointTrajectory joint_trajectory;

osrf_gear::GetMaterialLocations mat_bin_locations;
osrf_gear::LogicalCameraImage first_obj_info;

control_msgs::FollowJointTrajectoryAction joint_trajectory_as;


double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];


// Callbacks
void osrf_gearCallback(const osrf_gear::Order::ConstPtr& orders){
	order_vec.push_back(*orders);
}

void lc_bin1_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_bin_vec[0] = *msg ;
}
void lc_bin2_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_bin_vec[1] = *msg ;
} 
void lc_bin3_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_bin_vec[2] = *msg ;
}
void lc_bin4_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_bin_vec[3] = *msg ;
} 
void lc_bin5_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_bin_vec[4] = *msg ;
}
void lc_bin6_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_bin_vec[5] = *msg ;
}

void lc_agv1_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_agv_vec[0] = *msg ;
}
void lc_agv2_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_agv_vec[1] = *msg ;
}

void lc_fault1_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_fault_vec[0] = *msg ;
}
void lc_fault2_Callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){ 
lc_fault_vec[1] = *msg ;
}

void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
joint_states = *msg;
}



int main(int argc, char **argv)
  {
  ros::init(argc, argv, "subscriber_node");

  ros::NodeHandle n;
  
  order_vec.clear();
  lc_bin_vec.clear();
  lc_bin_vec.resize(6);
  lc_agv_vec.clear();
  lc_agv_vec.resize(2);
  lc_fault_vec.clear();
  lc_fault_vec.resize(2);
  

  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  ros::ServiceClient mat_locations= n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  
  ros::Subscriber order_subscriber = n.subscribe("/ariac/orders", 1000, osrf_gearCallback);
  
  ros::Subscriber lc_bin1 = n.subscribe("/ariac/logical_camera_bin1",1000,lc_bin1_Callback);
  ros::Subscriber lc_bin2 = n.subscribe("/ariac/logical_camera_bin2",1000,lc_bin2_Callback);
  ros::Subscriber lc_bin3 = n.subscribe("/ariac/logical_camera_bin3",1000,lc_bin3_Callback);
  ros::Subscriber lc_bin4 = n.subscribe("/ariac/logical_camera_bin4",1000,lc_bin4_Callback);
  ros::Subscriber lc_bin5 = n.subscribe("/ariac/logical_camerrostopia_bin5",1000,lc_bin5_Callback);
  ros::Subscriber lc_bin6 = n.subscribe("/ariac/logical_camera_bin6",1000,lc_bin6_Callback);
  
  ros::Subscriber lc_agv1 = n.subscribe("/ariac/logical_camera_agv1",1000,lc_agv1_Callback);
  ros::Subscriber lc_agv2 = n.subscribe("/ariac/logical_camera_agv2",1000,lc_agv2_Callback);
  
  ros::Subscriber lc_fault1 = n.subscribe("/ariac/quality_control_sensor_1",1000,lc_fault1_Callback);
  ros::Subscriber lc_fault2 = n.subscribe("/ariac/quality_control_sensor_2",1000,lc_fault2_Callback);
  
  // Need _h here for some reason - fixes the error of "class ros::Subscriber has no member named 'position' "
  ros::Subscriber joint_states_h = n.subscribe("/ariac/arm1/joint_states",1000, joint_states_Callback);
 
  ros::ServiceClient request_bin = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  service_call_succeeded = begin_client.call(begin_comp);
  
  // Instantiate action server client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);

	if (service_call_succeeded == 0){
	  ROS_ERROR("Competition service call failed! Goodness Gracious!!");
          ros::shutdown();
    }
  	else {
  		if(begin_comp.response.success){
  		ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
          	}
  		else{
	  	ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
	  		if(strcmp(begin_comp.response.message.c_str(), "cannot start if not in 'init' state")==0){
              			ros::shutdown();
            		}
  		}
  	}
  	
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown	
 
  while(ros::ok()){
  
	  if(order_vec.size() > 0){
	       
	       mat_bin_locations.request.material_type = order_vec[0].shipments[0].products[0].type;
	       
	       osrf_gear::StorageUnit obj_location;
	       
	       // find which bin the part is in and make sure it is not the belt	       
	       for (osrf_gear::StorageUnit unit : mat_bin_locations.response.storage_units){
	       	if (strcmp(unit.unit_id.c_str(),"belt") != 0){
	       		obj_location = unit;
	       	}
	       }

	  	if(request_bin.call(mat_bin_locations) == 0){
	  		ROS_ERROR("No Product and/or Order");
	  	}
	  	
	  	else{
			ROS_INFO("\nThe first object is: %s.\nIt can be found in: %s.", order_vec[0].shipments[0].products[0].type.c_str(), obj_location.unit_id.c_str());
			ROS_INFO_STREAM_THROTTLE(10, joint_states);
			
			std::string str = obj_location.unit_id.c_str();
			size_t i = 0;
			for ( ; i < str.length(); i++ ){ if ( isdigit(str[i]) ) break;}
			str = str.substr(i, str.length() - i );
			int bin_number_int = atoi(str.c_str());
			
			if (bin_number_int != 0){
				first_obj_info = lc_bin_vec[bin_number_int-1];
			}
			
			// print the pose of the models in the bin where the first product type was found
			for (int ii = 0;ii<first_obj_info.models.size(); ii++){	

				// check to make sure the product type found matched the desired
				if (strcmp(first_obj_info.models[ii].type.c_str(), order_vec[0].shipments[0].products[0].type.c_str()) == 0 ){
					if (first_run == true){
					ROS_INFO("\nProduct Type: %s.\nFound in: %s. \nIn the Pose: ", first_obj_info.models[ii].type.c_str(), obj_location.unit_id.c_str());
					ROS_WARN_STREAM(first_obj_info.models[ii].pose);
					first_run = false;
					}
				}
			}
	  	}
	} // end of checking order and getting first location
	
	// start to move arm
	while(/* ADD CONDITION FOR LATER */ true){
		geometry_msgs::PoseStamped waypoint;
		geometry_msgs::PoseStamped goal_pose;

		double T_way[4][4] = {{0.0, -1.0, 0.0, waypoint.pose.position.x - 0.2}, \
	      			          {0.0, 0.0, 1.0, waypoint.pose.position.y + 0.2}, \
	     			          {-1.0, 0.0, 0.0, waypoint.pose.position.z}, \
	      			          {0.0, 0.0, 0.0, 1.0}};
	      			      
		double T_des[4][4] = {{0.0, -1.0, 0.0, goal_pose.pose.position.x}, \
					          {0.0, 0.0, 1.0, goal_pose.pose.position.y}, \
					    	  {-1.0, 0.0, 0.0, goal_pose.pose.position.z + 0.1}, \
							  {0.0, 0.0, 0.0, 1.0}};
		
		int count = 0;
		int action_count = 0;

		// FIGURE OUT POSITION AFTER JOINT_STATE WORKS
		q_pose[0] = joint_states.position[1];
		q_pose[1] = joint_states.position[2];
		q_pose[2] = joint_states.position[3];
		q_pose[3] = joint_states.position[4];
		q_pose[4] = joint_states.position[5];
		q_pose[5] = joint_states.position[6];

		ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
		int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
		
		// Filling out joint trajectory header
		joint_trajectory.header.seq = count++;
	    joint_trajectory.header.stamp = ros::Time::now();
	    joint_trajectory.header.frame_id = "/base_link";
	
		// Set names of joints being used
		joint_trajectory.joint_names.clear();
		joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
		joint_trajectory.joint_names.push_back("shoulder_pan_joint");
		joint_trajectory.joint_names.push_back("shoulder_lift_joint");
		joint_trajectory.joint_names.push_back("elbow_joint");
		joint_trajectory.joint_names.push_back("wrist_1_joint");
		joint_trajectory.joint_names.push_back("wrist_2_joint");
		joint_trajectory.joint_names.push_back("wrist_3_joint");

		// Set start and end point
		joint_trajectory.points.resize(2);

		// Set start point to the current position from joint_states
		joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
		for(int indy = 0; indy < joint_trajectory.joint_names.size(); indy++){
			for(int indz = 0; indz < joint_states.name.size(); indz++) {
				if(joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
					joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
					break;
				}
			}
		}
	
		// Start immediately
		joint_trajectory.points[0].time_from_start = ros::Duration(0.0);






		joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
		actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
		ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());

		// Use first solution 
		int q_des_indx = 0;

		// Set end point for movement
		joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
		
		// joint_states.position[1] = linear arm actuator --> remove that for inverse kinematics
		joint_trajectory.points[1].positions[0] = joint_states.position[1];

		// The actuators are commanded in an odd order, enter the joint positions in the correct positions
		for (int indy = 0; indy < 6; indy++) {
			joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
		}

		// How long to take for movement
		joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
	



	
	} // end of inner while
	 ros::Duration(2).sleep(); 
	 
	 //ros::spinOnce(); 
	 
  } // end of while(ros::ok)
  ros::waitForShutdown();
  return 0;
}
