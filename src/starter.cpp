#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "ros/ros.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"


std_srvs::Trigger begin_comp;
//std_srvs::SetBool my_bool_var;
//my_bool_var.request.data = true;

bool first_run = true;

int service_call_succeeded;

std::vector<osrf_gear::Order> order_vec;

std::vector<osrf_gear::LogicalCameraImage> lc_bin_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_agv_vec;
std::vector<osrf_gear::LogicalCameraImage> lc_fault_vec;


osrf_gear::GetMaterialLocations mat_bin_locations;


osrf_gear::LogicalCameraImage first_obj_info;


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
  
  
  ros::ServiceClient request_bin = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
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
	  	
	  	//
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
	}
	    
	 ros::Duration(2).sleep(); 
	 
	 ros::spinOnce(); 
	  
  }
  ros::waitForShutdown();
  return 0;
}
