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
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <queue>


std_srvs::Trigger begin_comp;
//std_srvs::SetBool my_bool_var;
//my_bool_var.request.data = true;

bool first_run = true;


int service_call_succeeded;
int count = 0;

std::vector<osrf_gear::Order> order_vec;

std::vector<osrf_gear::LogicalCameraImage> lc_bin_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_agv_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_fault_vec;

sensor_msgs::JointState joint_states;

// trajectory_msgs::JointTrajectory joint_trajectory;
//control_msgs::FollowJointTrajectoryAction joint_trajectory_as;


osrf_gear::GetMaterialLocations mat_bin_locations;


osrf_gear::LogicalCameraImage first_obj_info;


void osrf_gearCallback(const osrf_gear::Order::ConstPtr& orders){
    ROS_WARN_STREAM("Received order:\n" << *orders); 
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

std::string find_bin_num(osrf_gear::Product product, ros::ServiceClient *request_bin){

    std::string bin_num;

    mat_bin_locations.request.material_type = product.type;
    osrf_gear::StorageUnit obj_location;
    
    // find which bin the part is in and make sure it is not the belt	       
    for (osrf_gear::StorageUnit unit : mat_bin_locations.response.storage_units){
    if (strcmp(unit.unit_id.c_str(),"belt") != 0){
        obj_location = unit;
    }
    }
    if(request_bin->call(mat_bin_locations) == 0){
        //ROS_ERROR("No Product and/or Order");
        bin_num = "N/A";
    }
    
    else{

    std::string str = obj_location.unit_id.c_str();
    size_t i = 0;
    for ( ; i < str.length(); i++ ){ if ( isdigit(str[i]) ) break;}
        str = str.substr(i, str.length() - i );
        bin_num = str.c_str();
    }

    ROS_INFO("\nThe object is: %s.\nIt can be found in: %s.", product.type.c_str(), obj_location.unit_id.c_str());

    return bin_num;
}

control_msgs::FollowJointTrajectoryAction move_to_joint_state(double q_desired[7], ros::Duration duration){

    ROS_INFO("Getting the trajectory...");
    trajectory_msgs::JointTrajectory joint_trajectory;

    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "/world";

    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");


    // Set a start and end point.
    joint_trajectory.points.resize(2);
    ROS_INFO("test0");
    // Set the start point to the current position of the joints from joint_states.
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
        for (int indz = 0; indz < joint_states.name.size(); indz++) {
            if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    ROS_INFO("test1");
    // sets the second point as the q_des input
    joint_trajectory.points[1].positions.resize(joint_trajectory.points.size());
    for (int indy = 0; indy < 7; indy++) {
         joint_trajectory.points[1].positions[indy] = q_desired[indy];
    }
    //joint_trajectory.points[1].positions[0] = q_desired[0];
    ROS_INFO("test2");
    // wait 0.5 seconds to start then run for the given duration
    joint_trajectory.points[0].time_from_start = ros::Duration(0.5);
    joint_trajectory.points[1].time_from_start = ros::Duration(0.5) + duration;

    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
    ROS_INFO("test3");
    joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
    joint_trajectory_as.action_goal.header.seq = count++;
    joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
    joint_trajectory_as.action_goal.header.frame_id = "/world";

    ROS_INFO("returning control message");
    return joint_trajectory_as;
    //actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); ROS_INFO("Action Server returned with status: %s", state.toString().c_str());
}

control_msgs::FollowJointTrajectoryAction move_linear(double y_position){

    // set the desired position to the current joint states except for the input to the linear actuator
    double q_des[7];
    q_des[0] = y_position;                 // linear_arm_actuator_joint
    q_des[1] = joint_states.position[3];   // shoulder_pan_joint
    q_des[2] = joint_states.position[2];   // shoulder_lift_joint
    q_des[3] = joint_states.position[0];   // elbow_joint
    q_des[4] = joint_states.position[4];   // wrist_1_joint
    q_des[5] = joint_states.position[5];   // wrist_2_joint
    q_des[6] = joint_states.position[6];   // wrist_3_joint

    control_msgs::FollowJointTrajectoryAction jtas;
    jtas = move_to_joint_state(q_des, ros::Duration(5.0));
    return jtas;

}

control_msgs::FollowJointTrajectoryAction go_home(){

    // set the desired position to the current joint states except for the input to the linear actuator
    double q_des[7];
    q_des[0] = joint_states.position[1];   // linear_arm_actuator_joint
    q_des[1] = 3.059876293438764;   // shoulder_pan_joint
    q_des[2] = 3.559122676382665;   // shoulder_lift_joint
    q_des[3] = 2.9079420271452174;   // elbow_joint
    q_des[4] = 3.8317176100661343;   // wrist_1_joint
    q_des[5] = -1.571491132250829;   // wrist_2_joint
    q_des[6] = -0.00027920417487159455;   // wrist_3_joint

    control_msgs::FollowJointTrajectoryAction jtas;
    jtas = move_to_joint_state(q_des, ros::Duration(5.0));
    return jtas;

}

control_msgs::FollowJointTrajectoryAction move_arm(geometry_msgs::PoseStamped desired_pose){

    double T_des[4][4] = {{0.0, -1.0, 0.0, desired_pose.pose.position.x}, \
						  {0.0, 0.0, 1.0, desired_pose.pose.position.y}, \
						  {-1.0, 0.0, 0.0, desired_pose.pose.position.z}, \
						  {0.0, 0.0, 0.0, 1.0}};

    double q_des[8][6];

    control_msgs::FollowJointTrajectoryAction jt_as;

    // third arguement could be the pose of the part later
    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des, 0.0);

    if (num_sols ==0){
        ROS_ERROR("No Inverse Kinematic Solutions Found");
        ROS_WARN("Going to home state...");
        jt_as = go_home();
    }
    else{
        ROS_INFO("Found a solution to inverse kinematics");
        double q_sol[7];
        q_sol[0] = joint_states.position[1]; // linear actuator position
        for( int i=1; i<7; i++){
            q_sol[i] = q_des[0][i-1];
        }
        jt_as = move_to_joint_state(q_sol, ros::Duration(10.0));
    }

    return  jt_as;

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
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
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

  ros::Subscriber joint_states_h = n.subscribe("/ariac/arm1/joint_states",1000, joint_states_Callback);
  
  
  ros::ServiceClient request_bin = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  service_call_succeeded = begin_client.call(begin_comp);

  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);

  
  
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
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);

 ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok()){
    sleep(10); // just giving everything time to finish setting up


    //ROS_INFO("in while");

	  if(!order_vec.empty()){


        osrf_gear::Order current_order = order_vec.at(0);
        //order_vec.erase(order_vec.begin());
        
            geometry_msgs::PoseStamped bin,robot_base, des_pose;
            geometry_msgs::TransformStamped tf, tf_2;

            /*
            Order of joint_states:
                - elbow_joint
                - linear_arm_actuator_joint
                - shoulder_lift_joint
                - shoulder_pan_joint
                - wrist_1_joint
                - wrist_2_joint
                - wrist_3_joint
                - vacuum_gripper_joint
            */

            for (osrf_gear::Shipment shipment: current_order.shipments){
                for(osrf_gear::Product product: shipment.products){

                    std::string bin_num = find_bin_num(product, &request_bin);

                    if (bin_num == "N/A" || bin_num.empty()){
                        ROS_ERROR("The bin for the part was not found");
                    }
                    else{
                        
                        int bin_num_int;
                        
                        bin_num_int = stoi(bin_num);
                        first_obj_info = lc_bin_vec[bin_num_int-1];
                        

                        const std::string frame_name = "logical_camera_bin" + bin_num + "_frame";

                        try{
                            tf = tfBuffer.lookupTransform("arm1_linear_arm_actuator", frame_name, ros::Time(0.0), ros::Duration(1.0));
                            ROS_DEBUG("Transform to [%s] from [%s]", tf.header.frame_id.c_str(),
						    tf.child_frame_id.c_str());
                        } 
                        catch (tf2::TransformException &ex) {
						    ROS_ERROR("%s", ex.what());
					    }
                        
                        bin.pose.position.x = 0.0;
                        bin.pose.position.y = 0.0;
                        bin.pose.position.z = 0.0;

                        tf2::doTransform(bin,robot_base,tf);
                        
                        double move_base_y = 0.0;
                        if (bin_num_int == 6){
                            move_base_y = robot_base.pose.position.y - 1.039;
                        }
                        else {
                            move_base_y = robot_base.pose.position.y + 1.039;
                        }

                        control_msgs::FollowJointTrajectoryAction jt_as;

                        // moving the base of the robot
                        jt_as = move_linear(move_base_y);
                        ROS_INFO("starting to move");
                        actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); ROS_INFO("Action Server returned with status: %s", state.toString().c_str());
                        ROS_INFO("done moving");
                        

                        /*
                        // finding the desired joint angles
                        osrf_gear::LogicalCameraImage models_in_bin = lc_bin_vec[bin_num_int-1];
                        for (int i = 0; i<models_in_bin.models.size(); i++){
                            geometry_msgs::PoseStamped part_pose_local, part_pose_arm_frame;

                            // Get the pose of the part in the bin
                            part_pose_local.pose.position.x = models_in_bin.models[i].pose.position.x;
                            part_pose_local.pose.position.y = models_in_bin.models[i].pose.position.y;
                            part_pose_local.pose.position.z = models_in_bin.models[i].pose.position.z;

                            try{
                                // find the transform from the part pose in the bin to the base of the robot arm
                                tf_2 = tfBuffer.lookupTransform("arm1_base_link", frame_name, ros::Time(0.0), ros::Duration(1.0));
                                ROS_DEBUG("Transform to [%s] from [%s]", tf_2.header.frame_id.c_str(),
                                tf.child_frame_id.c_str());
                            } 
                            catch (tf2::TransformException &ex) {
                                ROS_ERROR("%s", ex.what());
                            }

                            // do the transformation
                            tf2::doTransform(part_pose_local, part_pose_arm_frame, tf_2);
                            
                            control_msgs::FollowJointTrajectoryAction jt_as_2, jt_as_3;

                            // moving the base of the robot
                            jt_as_2 = move_arm(part_pose_arm_frame);
                            actionlib::SimpleClientGoalState state2 = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); ROS_INFO("Action Server returned with status: %s", state.toString().c_str());

                            jt_as_3 = go_home();
                            actionlib::SimpleClientGoalState state3 = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); ROS_INFO("Action Server returned with status: %s", state.toString().c_str());
                           
                        }
 */

                    }
                   

                }
            }
        }


            /* 
            for # of shipments
                for part in shipment
                    move the base of the robot
                    solve inverse kinematics
                    move to part


                    vacuum gripper on
                    move to home postion
                    move linear actuator
                    move to desired placement in argv bins
                    vacuum gripper off
                end
            end
            */

        loop_rate.sleep();
        count++;


    }

    return 0;
     
}
  
