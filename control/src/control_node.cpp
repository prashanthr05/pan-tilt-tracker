#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <control/dynamixel.h>
#include <control/dxl_hal.h> 
#include <math.h>
#include <stdio.h>
#include <iostream>




// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46

// Default setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define DEFAULT_PAN_ID		5
#define DEFAULT_TILT_ID		7


using namespace ros;
using namespace std;

double step=1024;
double servo_angle=2.6;
double cam_x=0.32;
double cam_y=0.24;
double camx_postoangle=0.43;
double camy_postoangle=0.21;
int pan_pos=0;
int tilt_pos=0;
int Moving,CommStatus;

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);


void pancallback(geometry_msgs::PointStamped pan_point)
{


	cout<<"----------------------------------"<<endl;
	cout<<"X POINT RECEIVED : "<<pan_point.point.y<<endl;
	double pan = (-pan_point.point.y/cam_x)*camx_postoangle;
	pan_pos =int( (pan/servo_angle)*(step/2) );
	pan_pos=512+pan_pos;
	cout<<"Pan Position in servo steps :"<<pan_pos<<endl;

	dxl_write_word( DEFAULT_PAN_ID, P_GOAL_POSITION_L,pan_pos);
	
	
			// Check moving done
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


	cout<<"Y POINT RECEIVED : "<<pan_point.point.x<<endl;
	double tilt = (pan_point.point.x/cam_y)*camy_postoangle;
	tilt_pos =int( (tilt/servo_angle)*(step/2) );
	tilt_pos=196+tilt_pos;
	cout<<"Tilt Position in servo steps :"<<tilt_pos<<endl;

	dxl_write_word( DEFAULT_TILT_ID, P_GOAL_POSITION_L,tilt_pos);
	
	
			// Check moving done
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
	
	
}





int main(int argc, char** argv)
{
	ros::init(argc,argv,"control");
	NodeHandle nh;
	ros::Rate loop_rate(10);
	Subscriber pan_sub = nh.subscribe<geometry_msgs::PointStamped>("/target_point",1,pancallback);

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

	


	while(ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
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

