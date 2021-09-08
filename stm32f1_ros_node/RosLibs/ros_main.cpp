#include "ros_main.h"
#include "main.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

// ROS Node Handler
ros::NodeHandle nh;

uint32_t tTime[10] = {0};
uint32_t t;

// ROS stm32_status Publisher
char stm32_cmd_vel_ready_msg[]= "STM32_cmd_vel_OK";
std_msgs::String str_msg;
ros::Publisher stm32_status_pub("/stm32/system_up", &str_msg);


// ROS motor encoders Publisher
geometry_msgs::Vector3 encoders_data;
ros::Publisher encoder_pub("/stm32/encoders",&encoders_data);

// Callback of cmd_vel
void cmd_vel_msg(const geometry_msgs::Twist& msg)
{
	

	
}


// ROS cmd_vel Subscriber
ros::Subscriber<geometry_msgs::Twist> velocity_sub("/cmd_vel", &cmd_vel_msg);

void setup(void)
{
  nh.initNode();
	// Call ROS Master to keep a registry of this publisher
  nh.advertise(stm32_status_pub);
	nh.advertise(encoder_pub);
	// Subscribe cmd_vel
	nh.subscribe(velocity_sub);
}


void loop(void)
{
  // Life LED	
	HAL_GPIO_TogglePin(ONBOARD_LED_GPIO_Port,ONBOARD_LED_Pin);
	// Publish encoder vectors
	encoders_data.x = 0;
	encoders_data.y = 0;
	encoder_pub.publish(&encoders_data);
	
	nh.spinOnce();
	HAL_Delay(500);
}
