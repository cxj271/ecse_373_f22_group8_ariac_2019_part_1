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
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>


std_srvs::Trigger begin_comp;
//std_srvs::SetBool my_bool_var;
//my_bool_var.request.data = true;

bool first_run = true;
osrf_gear::VacuumGripperState vacuum_state;


int service_call_succeeded;
int count = 0;

std::vector<osrf_gear::Order> order_vec;

std::vector<osrf_gear::LogicalCameraImage> lc_bin_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_agv_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_fault_vec;

sensor_msgs::JointState joint_states;

// trajectory_msgs::JointTrajectory joint_trajectory;
//control_msgs::FollowJointTrajectoryAction joint_trajectory_as;


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

void vacuum_Callback(const osrf_gear::VacuumGripperState::ConstPtr& msg){
    vacuum_state = *msg;
}

std::string find_bin_num(std::string material, ros::ServiceClient *client){
    ROS_INFO("trying to find a bin...");
    std::string bin_num_string;
    osrf_gear::GetMaterialLocations mat_bin_location;
    mat_bin_location.request.material_type = material;
    bool found_bin = client->call(mat_bin_location);
    if (!found_bin){
        ROS_ERROR("No Product and/or Order");
        bin_num_string = "N/A";
    }
    else{
        for (osrf_gear::StorageUnit unit : mat_bin_location.response.storage_units){
            if (strcmp(unit.unit_id.c_str(), "belt") != 0){
                std::string str = unit.unit_id.c_str();
                size_t i=0;
                for ( ; i < str.length(); i++ ){ if ( isdigit(str[i]) ) break;}
                str = str.substr(i, str.length() - i );
                bin_num_string = str.c_str();
                ROS_INFO("\nThe object is: %s.\nIt can be found in: %s.", material.c_str(), unit.unit_id.c_str());

                break;
            }
        }

    }   
    return bin_num_string;
}

void move_to_joint_state(double q_desired[7], ros::Duration duration, trajectory_msgs::JointTrajectory *p_joint_trajectory){
    //ROS_INFO("Getting the trajectory...");

    p_joint_trajectory->header.seq = count++;
    p_joint_trajectory->header.stamp = ros::Time::now();
    p_joint_trajectory->header.frame_id = "/world";

    p_joint_trajectory->joint_names.clear();
    p_joint_trajectory->joint_names.push_back("linear_arm_actuator_joint");
    p_joint_trajectory->joint_names.push_back("shoulder_pan_joint");
    p_joint_trajectory->joint_names.push_back("shoulder_lift_joint");
    p_joint_trajectory->joint_names.push_back("elbow_joint");
    p_joint_trajectory->joint_names.push_back("wrist_1_joint");
    p_joint_trajectory->joint_names.push_back("wrist_2_joint");
    p_joint_trajectory->joint_names.push_back("wrist_3_joint");


    // Set a start and end point.
    p_joint_trajectory->points.resize(2);
    // Set the start point to the current position of the joints from joint_states.
    p_joint_trajectory->points[0].positions.resize(p_joint_trajectory->joint_names.size());
    for (int indy = 0; indy < p_joint_trajectory->joint_names.size(); indy++) {
        for (int indz = 0; indz < joint_states.name.size(); indz++) {
            if (p_joint_trajectory->joint_names[indy] == joint_states.name[indz]) {
                p_joint_trajectory->points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    // sets the second point as the q_des input
    p_joint_trajectory->points[1].positions.resize(p_joint_trajectory->joint_names.size());
    for (int indy = 0; indy < 7; indy++) {
         p_joint_trajectory->points[1].positions[indy] = q_desired[indy];
    }
    //joint_trajectory.points[1].positions[0] = q_desired[0];
    // wait 0.5 seconds to start then run for the given duration
    p_joint_trajectory->points[0].time_from_start = ros::Duration(0.1);
    p_joint_trajectory->points[1].time_from_start = duration;

    //actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); ROS_INFO("Action Server returned with status: %s", state.toString().c_str());
}

void move_linear(double y_position,trajectory_msgs::JointTrajectory *p_joint_trajectory){

    // set the desired position to the current joint states except for the input to the linear actuator
    double q_des[7];
    q_des[0] = y_position;                 // linear_arm_actuator_joint
    q_des[1] = joint_states.position[3];   // shoulder_pan_joint
    q_des[2] = joint_states.position[2];   // shoulder_lift_joint
    q_des[3] = joint_states.position[0];   // elbow_joint
    q_des[4] = joint_states.position[4];   // wrist_1_joint
    q_des[5] = joint_states.position[5];   // wrist_2_joint
    q_des[6] = joint_states.position[6];   // wrist_3_joint

    move_to_joint_state(q_des, ros::Duration(5.0), p_joint_trajectory);
}

void lift_part(trajectory_msgs::JointTrajectory *p_joint_trajectory){

    // set the desired position to the current joint states except for the input to the linear actuator
    double q_des[7];
    q_des[0] = joint_states.position[1];                 // linear_arm_actuator_joint
    q_des[1] = joint_states.position[3];   // shoulder_pan_joint
    q_des[2] = joint_states.position[2] - 0.3;   // shoulder_lift_joint
    q_des[3] = joint_states.position[0];   // elbow_joint
    q_des[4] = joint_states.position[4];   // wrist_1_joint
    q_des[5] = joint_states.position[5];   // wrist_2_joint
    q_des[6] = joint_states.position[6];   // wrist_3_joint

    move_to_joint_state(q_des, ros::Duration(2.0), p_joint_trajectory);
}

void drop_part(trajectory_msgs::JointTrajectory *p_joint_trajectory){

    // set the desired position to the current joint states except for the input to the linear actuator
    double q_des[7];
    q_des[0] = joint_states.position[1];                 // linear_arm_actuator_joint
    q_des[1] = joint_states.position[3];   // shoulder_pan_joint
    q_des[2] = joint_states.position[2] + 0.25;   // shoulder_lift_joint
    q_des[3] = joint_states.position[0];   // elbow_joint
    q_des[4] = joint_states.position[4];   // wrist_1_joint
    q_des[5] = joint_states.position[5];   // wrist_2_joint
    q_des[6] = joint_states.position[6];   // wrist_3_joint

    move_to_joint_state(q_des, ros::Duration(2.0), p_joint_trajectory);
}


void go_home(trajectory_msgs::JointTrajectory *p_joint_trajectory){

    // set the desired position to the current joint states except for the input to the linear actuator
    double q_des[7];
    q_des[0] = joint_states.position[1];   // linear_arm_actuator_joint
    q_des[1] = 3.059876293438764;   // shoulder_pan_joint
    q_des[2] = 3.559122676382665;   // shoulder_lift_joint
    q_des[3] = 2.9079420271452174;   // elbow_joint
    q_des[4] = 3.8317176100661343;   // wrist_1_joint
    q_des[5] = -1.571491132250829;   // wrist_2_joint
    q_des[6] = -0.00027920417487159455;   // wrist_3_joint

    
    move_to_joint_state(q_des, ros::Duration(8.0), p_joint_trajectory);
    //return jtas;

}

bool move_arm(geometry_msgs::PoseStamped desired_pose,trajectory_msgs::JointTrajectory *p_joint_trajectory){

    double T_des[4][4] = {{0.0, -1.0, 0.0, desired_pose.pose.position.x}, \
						  {0.0, 0.0, 1.0, desired_pose.pose.position.y}, \
						  {-1.0, 0.0, 0.0, desired_pose.pose.position.z}, \
						  {0.0, 0.0, 0.0, 1.0}};


    double q_des[8][6];

    // third arguement could be the pose of the part later
    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des, 0.0);

    int q_sol_ind = 10; //start at some impossible value
        for (int i = 0; i < num_sols; i++){
            if (q_des[i][1] > 4.5){
                if (q_des[i][3] > 2.0){

                    //if (q_des[i][4] < 3.0){
                        ROS_INFO("Found a solution to inverse kinematics");
                        q_sol_ind = i;
                        break; //found a valid solution
                    //}

                }
            }
        }
    if (q_sol_ind == 10){
        // solution foud but the arm has to bend backwards or something that doesn't actually work
        num_sols = 0;
    }
    

    if (num_sols == 0){
        ROS_ERROR("No Inverse Kinematic Solutions Found");
        return false;
        //ROS_WARN("Going to home state...");
        //go_home(p_joint_trajectory);
    }
    else{
        //ROS_INFO("Found a solution to inverse kinematics");

        double q_sol[7];
        q_sol[0] = joint_states.position[1]; // linear actuator position
        for( int i=1; i<7; i++){
            q_sol[i] = q_des[q_sol_ind][i-1];
        }
        //ROS_WARN_STREAM(q_sol[0]);
        move_to_joint_state(q_sol, ros::Duration(3.0),p_joint_trajectory);
        return true;
    }  

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
  ros::Subscriber vacuum_state_subcriber = n.subscribe("/ariac/arm1/gripper/state", 1000, vacuum_Callback);
  
  ros::ServiceClient vacuum_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");

  ros::ServiceClient request_bin = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  request_bin.waitForExistence();

  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);

ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
begin_client.waitForExistence();
service_call_succeeded = begin_client.call(begin_comp);
  
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
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok()){
    sleep(10); // just giving everything time to finish setting up


    //ROS_INFO("in while");

	  if(!order_vec.empty()){


        osrf_gear::Order current_order = order_vec.at(0);
        //order_vec.erase(order_vec.begin());
        
            geometry_msgs::PoseStamped bin,robot_base, des_pose;
            geometry_msgs::TransformStamped tf, tf_2, tf_3;

            for (osrf_gear::Shipment shipment: current_order.shipments){
                for(osrf_gear::Product product: shipment.products){

                    std::string bin_number = find_bin_num(product.type.c_str(), &request_bin);

                    if (bin_number == "N/A" || bin_number.empty()){
                        ROS_ERROR("The bin for the part was not found");
                    }
                    else{
                        
                        int bin_num_int;
                        
                        bin_num_int = stoi(bin_number);
                        
                        first_obj_info = lc_bin_vec[bin_num_int-1];
                        

                        const std::string frame_name = "logical_camera_bin" + bin_number + "_frame";

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
                        double linear_offset = 1.039;

                        control_msgs::FollowJointTrajectoryAction jt_as;
                        trajectory_msgs::JointTrajectory jt;                                               
                        
                        // finding the desired joint angles
                        osrf_gear::LogicalCameraImage models_in_bin = lc_bin_vec[bin_num_int-1];
                        for (int i = 0; i<models_in_bin.models.size(); i++){

                            if (bin_num_int == 6){
                                move_base_y = robot_base.pose.position.y - linear_offset;
                            }
                            else {
                                move_base_y = robot_base.pose.position.y + linear_offset;
                            }

                            // moving the base of the robot
                            move_linear(move_base_y, &jt);
                            jt_as.action_goal.goal.trajectory = jt;
                            jt_as.action_goal.header.seq = count++;
                            jt_as.action_goal.header.stamp = ros::Time::now();
                            jt_as.action_goal.header.frame_id = "/world";

                            ROS_INFO("starting to move linear actuator");
                            actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); 
                            ROS_INFO("Action Server returned with status: %s", state.toString().c_str());
                            ROS_INFO("done moving linear actuator");
                            sleep(2);

                            geometry_msgs::PoseStamped part_pose_local, part_pose_arm_frame, part_pose_linear_actuator;

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

                            // TODO: make move_arm a bool to check if inverse kinematics exist and only try to move if it does
                            bool IK_found = move_arm(part_pose_arm_frame, &jt);

                            if (!IK_found){

                                for (int ii = 0; ii < 6; ii++){
                                    //try moving a bit closer
                                    ROS_INFO("Let's try moving a bit closer");
                                    if (bin_num_int == 6){
                                    move_base_y += 0.05;
                                    }
                                    else {
                                    move_base_y -= 0.05;
                                    }
                                    move_linear(move_base_y, &jt);
                                    jt_as.action_goal.goal.trajectory = jt;
                                    jt_as.action_goal.header.seq = count++;
                                    jt_as.action_goal.header.stamp = ros::Time::now();
                                    jt_as.action_goal.header.frame_id = "/world";
                                    //ROS_INFO("starting to move linear actuator");
                                    actionlib::SimpleClientGoalState state10 = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); 
                                    ROS_INFO("Action Server returned with status: %s", state10.toString().c_str());
                                    //ROS_INFO("done moving linear actuator");
                                    sleep(2);

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

                                    IK_found = move_arm(part_pose_arm_frame, &jt);
                                    ROS_WARN_STREAM(IK_found);
                                    if (IK_found){
                                        break;
                                    }
                                }

                            }

                            IK_found = move_arm(part_pose_arm_frame, &jt);

                            
                            if (IK_found == true){
                                jt_as.action_goal.goal.trajectory = jt;
                                jt_as.action_goal.header.seq = count++;
                                jt_as.action_goal.header.stamp = ros::Time::now();
                                jt_as.action_goal.header.frame_id = "/world";

                                ROS_INFO("Moving Arm to part %i", i);
                                actionlib::SimpleClientGoalState state2 = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); 
                                ROS_INFO("Action Server returned with status: %s", state2.toString().c_str());
                                ROS_INFO("Done moving Arm to part");
                                sleep(5);

/*
                                osrf_gear::VacuumGripperControl vac_control; 
                                vac_control.request.enable = true;
                                vacuum_client.call(vac_control);
                                ROS_INFO("gripper on");

                                ros::Duration(3.0).sleep();
                                if (vacuum_state.attached){
                                    ROS_INFO("Vacuum has picked up the part");
                                */
                                
                                    // lift_part(&jt);
                                    // jt_as.action_goal.goal.trajectory = jt;
                                    // jt_as.action_goal.header.seq = count++;
                                    // jt_as.action_goal.header.stamp = ros::Time::now();
                                    // jt_as.action_goal.header.frame_id = "/world";

                                    // ROS_INFO("Lifting part up");
                                    // actionlib::SimpleClientGoalState state3 = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); 
                                    // ROS_INFO("Action Server returned with status: %s", state3.toString().c_str());
                                    // ROS_INFO("Done lifting part");
                                    // sleep(2);

                                    // drop_part(&jt);
                                    // jt_as.action_goal.goal.trajectory = jt;
                                    // jt_as.action_goal.header.seq = count++;
                                    // jt_as.action_goal.header.stamp = ros::Time::now();
                                    // jt_as.action_goal.header.frame_id = "/world";

                                    // ROS_INFO("Putting part down");
                                    // actionlib::SimpleClientGoalState state4 = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); 
                                    // ROS_INFO("Action Server returned with status: %s", state4.toString().c_str());
                                    // ROS_INFO("Done putting part down");
                                    // sleep(2);

                            //         ROS_INFO("dropping part");
                            //         vac_control.request.enable = false;
                            //         vacuum_client.call(vac_control);
                            //     }
                            //     else{
                            //         ROS_ERROR("Vacuum failed");
                            //         vac_control.request.enable = false;
                            //         vacuum_client.call(vac_control);
                            //     }

                             }


                            go_home(&jt);
                            jt_as.action_goal.goal.trajectory = jt;
                            jt_as.action_goal.header.seq = count++;
                            jt_as.action_goal.header.stamp = ros::Time::now();
                            jt_as.action_goal.header.frame_id = "/world";

                            ROS_INFO("Moving to home state");
                            actionlib::SimpleClientGoalState state5 = trajectory_as.sendGoalAndWait(jt_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0)); 
                            ROS_INFO("Action Server returned with status: %s", state5.toString().c_str());
                            ROS_INFO("Done moving to home state");
                            sleep(5);
                        }

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
  
