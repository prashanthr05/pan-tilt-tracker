#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//#include <tf/transform_broadcaster.h>

#include <iostream>
using namespace std;
using namespace cv;
using namespace ros;
//image publisher
image_transport::Publisher image_pub;

Publisher objpos_pub;



//variables for the position co-ordinates of the moment of tracked object
float lastX = -1;
float lastY = -1;

geometry_msgs::PointStamped pose;

//callback function that is used to subscribe to a raw image which is processed in order to track
//a yellow colored object and publishes the processed image
void imagecallback(const sensor_msgs::ImageConstPtr& rosImage)
{
	cv_bridge::CvImagePtr cvImage;
	try
	{
		cvImage = cv_bridge::toCvCopy(rosImage,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'BGR8'",rosImage->encoding.c_str());
}

	Mat img = cvImage->image;
	Mat hsv,imgthr;
	Mat track = Mat::zeros(img.size(),CV_8UC3);

	//converting rgb to hsv format and thresholding to yellow color
	//morphological opening and closing is done in order to remove extra patches
	//and fill any holes in the image respectively
	cvtColor(img,hsv,COLOR_BGR2HSV);
	inRange(hsv,Scalar(20,100,100),Scalar(30,255,255),imgthr);
			//morphological opening
			erode(imgthr,imgthr,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
			dilate(imgthr,imgthr,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
			//morphological closing
			dilate(imgthr,imgthr,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
			erode(imgthr,imgthr,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
			GaussianBlur(imgthr,imgthr,Size(5,5),0,0);

			//the spatial moments of the object is found to get the centre of mass co-ordinates
			//which we use to track the object
			//the function moments calculates upto third order moments of a polygon or a rasterized shape
			Moments m = moments(imgthr);


			double M01 = m.m01;
			double M10 = m.m10;
			double Area = m.m00;

			if(Area > 100)
			{
				//posX and posY are the x and y co-ordinates for the centroid of the object
				float posX = M10/Area;
				float posY = M01/Area;
				if(lastX >=0 && lastY >=0 && posX >=0 && posY >=0)
				{
					line(track,Point(posX,posY),Point(lastX,lastY),Scalar(0,0,255),2);
				}
				lastX = posX;
				lastY = posY;

				pose.point.x = lastY;
				pose.point.y = lastX;
				pose.header.frame_id="/fixed_camera";
				pose.header.stamp = ros::Time::now();
				objpos_pub.publish(pose);


			}

			img = img + track;

			cvImage->image = img;

	namedWindow("Image Processed");
	imshow("Image Processed",cvImage->image);
	waitKey(3);
	image_pub.publish(cvImage->toImageMsg());

	//cout<<"LastX : "<<lastY*0.001-0.24<<"LastY :"<<lastX*0.001-0.32<<endl;
	static tf::TransformBroadcaster b;
        tf::Transform camera_frame;
        camera_frame.setOrigin( tf::Vector3(lastY*0.001-0.24, lastX*0.001-0.32, -0.5));
	//camera_frame.setOrigin( tf::Vector3(lastY-240, lastX-320, -0.5));
        tf::Quaternion q;
       	q.setRPY(0.0, 0.0, 0.0);
        camera_frame.setRotation(q);
       	b.sendTransform(tf::StampedTransform(camera_frame, ros::Time::now(), pose.header.frame_id,"/target_frame"));
	
	
		

}

int main(int argc, char **argv)
{
	

	//initialising Ros
	init(argc,argv,"vision");
	
	//declaring nh as a nodehandle to handle the messages and topics coming in and going out of the node
	NodeHandle nh;
	Rate rate(10);
	
	//imgtrans is an instance of image transport in order to handle images as ros messages
	image_transport::ImageTransport imgtrans(nh);
	namedWindow("Image Processed",1);

	
	//this vision node subscribes to the image_raw topic coming from the camera
	image_transport::Subscriber image_sub = imgtrans.subscribe("/camera/image_raw",1,imagecallback);
	destroyWindow("Image Processed");
	
	//and publishes the processed image to the topic image_processed
	image_pub = imgtrans.advertise("/camera/image_processed",1);
	objpos_pub = nh.advertise<geometry_msgs::PointStamped>("/object_pose",1);
	
	
	while(ros::ok()){
		spinOnce();
		rate.sleep();

	}


}




