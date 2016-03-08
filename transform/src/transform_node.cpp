#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>

#include <tf/transform_listener.h>

#include <iostream>

using namespace std;
using namespace ros;


geometry_msgs::PointStamped target_point,pan_joint,tilt_joint;
Publisher target_pub,pan_joint_pub,tilt_joint_pub;




int main(int argc , char** argv)
{	
	ros::init(argc,argv,"transform");
	
	ros::NodeHandle nh;
	
	target_pub = nh.advertise<geometry_msgs::PointStamped>("/target_point",1);
	pan_joint_pub = nh.advertise<geometry_msgs::PointStamped>("/pan_joint",1);
	tilt_joint_pub = nh.advertise<geometry_msgs::PointStamped>("/tilt_joint",1);
	
	tf::TransformListener target_listener;
	tf::TransformListener pan_joint_listener;
	tf::TransformListener tilt_joint_listener;

	tf::StampedTransform target_tracking;	
	tf::StampedTransform pan_joint_transform;
	tf::StampedTransform tilt_joint_transform;
	
	while(ros::ok()){

	
  	
	try{
		target_listener.waitForTransform("/fixed_camera","/target_frame",ros::Time(0),ros::Duration(10));
		target_listener.lookupTransform("/fixed_camera","/target_frame",ros::Time(0),target_tracking);
		
	    }

	 catch (tf::TransformException ex){
     		 ROS_ERROR("%s",ex.what());
    	   }
		target_point.header.seq = 10;
		target_point.header.stamp = ros::Time(0);
		target_point.header.frame_id = "/fixed_camera";
		target_point.point.x = target_tracking.getOrigin().x();
		target_point.point.y = target_tracking.getOrigin().y();
		target_point.point.z = target_tracking.getOrigin().z();
		
		target_pub.publish(target_point);


	
	/*try{
		pan_joint_listener.waitForTransform("/base_link","/pan_link",ros::Time(0),ros::Duration(10));
		pan_joint_listener.lookupTransform("/base_link","/pan_link",ros::Time(0),pan_joint_transform);
	    }

	 catch (tf::TransformException ex){
     		 ROS_ERROR("%s",ex.what());
    	   }
		pan_joint.header.seq = 10;
		pan_joint.header.stamp = ros::Time(0);
		pan_joint.header.frame_id = "/base_link";
		pan_joint.point.x = pan_joint_transform.getOrigin().x();
		pan_joint.point.y = pan_joint_transform.getOrigin().y();
		pan_joint.point.z = pan_joint_transform.getOrigin().z();
		
		pan_joint_pub.publish(pan_joint);


	
	try{
		tilt_joint_listener.lookupTransform("/base_link","/tilt_link",ros::Time(0),ros::Duration(10));
		tilt_joint_listener.lookupTransform("/base_link","/tilt_link",ros::Time(0),tilt_joint_transform);
	    }

	 catch (tf::TransformException ex){
     		 ROS_ERROR("%s",ex.what());
    	   }
		tilt_joint.header.seq = 10;
		tilt_joint.header.stamp = ros::Time(0);
		tilt_joint.header.frame_id = "/base_link";
		tilt_joint.point.x = tilt_joint_transform.getOrigin().x();
		tilt_joint.point.y = tilt_joint_transform.getOrigin().y();
		tilt_joint.point.z = tilt_joint_transform.getOrigin().z();
		
		tilt_joint_pub.publish(tilt_joint);*/
	

	}
	
	return 0;
}


	
	
