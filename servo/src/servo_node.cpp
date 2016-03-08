#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <servo/dynamixel.h>
#include <servo/dxl_hal.h> 


// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46

// Defulat setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define DEFAULT_PAN_ID		5
#define DEFAULT_TILT_ID		7

double step=1024;
double angle=2.61;

using namespace ros;
using namespace std;

ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;


double present_value_pan=0;
double present_value_tilt=0;


double pan_value=0;
double tilt_value=0;

double pan_conv=0;
double tilt_conv=0;


int Moving,CommStatus;
void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);		  
		
int main(int argc,char** argv){


	ros::init(argc,argv,"pantilt");
        ROS_INFO("node creation");
	ros::NodeHandle ntc;

	int baudnum = 1;
	int index = 0;
	int deviceIndex = 0;

	ROS_INFO( "\n\nRead/Write example for Linux\n\n" );
	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		ROS_INFO( "Failed to open USB2Dynamixel!\n" );
		ROS_INFO( "Press Enter key to terminate...\n" );
		getchar();
		return 0;
	}
	else
		ROS_INFO( "Succeed to open USB2Dynamixel!\n" );

       		

	

	//Publishing
	joint_pub=ntc.advertise<sensor_msgs::JointState>("/joint_states",1);

	



	
	while(ros::ok()){

			ROS_INFO( "INSIDE SERVO LOOP\n" );

			present_value_pan= dxl_read_word( DEFAULT_PAN_ID, P_PRESENT_POSITION_L );

		Moving = dxl_read_byte( DEFAULT_PAN_ID, P_MOVING );
			CommStatus = dxl_get_result();
			if( CommStatus == COMM_RXSUCCESS )
			{
				
				PrintErrorCode();
			}
			else
			{
				PrintCommStatus(CommStatus);
			}
		  	
			
		present_value_tilt= dxl_read_word( DEFAULT_TILT_ID, P_PRESENT_POSITION_L );
		Moving = dxl_read_byte( DEFAULT_TILT_ID, P_MOVING );
			CommStatus = dxl_get_result();
			if( CommStatus == COMM_RXSUCCESS )
			{
				
				PrintErrorCode();
			}
			else
			{
				PrintCommStatus(CommStatus);
			}
		  	
			
			
		
		
		
			cout<< "present_value_pan : " << present_value_pan<< " present_value_tilt : " << present_value_tilt << endl;

			
			
			pan_conv=(present_value_pan/step)*(2*angle);
			pan_value=pan_conv-angle;
			
			tilt_conv=(present_value_tilt/step)*(2*angle);
			tilt_value=tilt_conv-angle;
			
			cout<< "pan_value : " << pan_value << " tilt_value : " << tilt_value << endl;
			cout << "-------------------------" << endl;


			joint_state.header.stamp = ros::Time::now();
        		joint_state.name.resize(2);
       		        joint_state.position.resize(2);
        		joint_state.name[0] ="PAN_JOINT";
        		joint_state.position[0] = pan_value;
        		joint_state.name[1] ="TILT_JOINT";
        		joint_state.position[1] = -tilt_value;
       
        		//send the joint state 
        		joint_pub.publish(joint_state);

			static tf::TransformBroadcaster br;
        		tf::Transform new_frame;
        		new_frame.setOrigin( tf::Vector3(0, 0, 0));
        		tf::Quaternion q;
       			q.setRPY(0.0, 0.0, 0.0);
        		new_frame.setRotation(q);
       			br.sendTransform(tf::StampedTransform(new_frame, ros::Time::now(), "/world","/base_link"));
	
       }
	return 0;
}


// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}

