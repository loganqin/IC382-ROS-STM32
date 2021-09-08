#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265

// Basic ROS configuration
const char node_name[] = "encoder_to_odometry_publisher";      // node name
const char encoder_topic[] = "/stm32/encoders";                // publisher
const char odom_topic[] = "/odom";                             // subsriber
const char speed_topic[] = "/cmd_vel";                         // subscriber
const char source_link[] = "odom";                             // TF pair
const char target_link[] = "base_link";                        // TF pair
const int ros_update_hz = 1;                                   // in Hz


// Robot physical parameters
const double offset_distance_error = 3.580;                                                  // distance compensation
const double offset_theta_error = 0.18;                                                      // theta compensation
const double robot_wheel_seperation = 0.3;                                                   // in meter
const double robot_wheel_radius = 0.01;                                                     // in meter
const double encoder_per_rotation_m1 = 28911.0;                                              // encoder value per rotation of motor1 [from 65535 - 65305]
const double encoder_per_rotation_m2 = 28911.0;                                              // encoder value per rotation of motor2 [from 0 - 28911]
double distance_per_count_m1 = (double)(2*PI*robot_wheel_radius)/encoder_per_rotation_m1;    // Distance for an encoder pulse in m
double distance_per_count_m2 = (double)(2*PI*robot_wheel_radius)/encoder_per_rotation_m2 ;   // Distance for an encoder pulse in m

// Variables
geometry_msgs::Twist msg;
ros::Time current_time, last_time;  // ROS Time
double linear_x = 0;                // robot x
double linear_y = 0;                // robot y
double angular_z = 0;               // robot angular z

// Instantaneous variables
double dm1 = 0;
double dm2 = 0;
double dc = 0;
double dx = 0;
double dy = 0;
double dtheta = 0;
double dt = 0;

// Odometry data
double x = 0;
double y = 0;
double theta = 0;

// Previous value
double _x_tick = 0;
double _y_tick = 0;
double _theta = 0;


void VelocityCallback(const geometry_msgs::Twist& msg)
{
    linear_x = msg.linear.x;
    linear_y = msg.linear.y;
    angular_z = msg.angular.z;
}

void EncoderCallback(const geometry_msgs::Vector3::ConstPtr& encoder_ticks)
{

	         
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(encoder_topic, 100, EncoderCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 50);
    ros::Subscriber vel_sub = nh.subscribe(speed_topic,100,VelocityCallback);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate r(ros_update_hz);
    printf("Encoder To Odom Transformation started!\r\n");

    // Publish loop
    while (nh.ok())
    {
        // Only Yaw should be considered
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        // A TF should be setup between base_link and odom
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = source_link;
        odom_trans.child_frame_id = target_link;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // Send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // Publish odom message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = source_link;
        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.child_frame_id = target_link;
        odom.twist.twist.linear.x=dx/dt;
        odom.twist.twist.linear.y=dy/dt;
        odom.twist.twist.angular.z =dtheta/dt; 

        //printf("Position x = %f and Position y = %f ",x,y);
	
        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;
        ros::spinOnce();

    }
    return 0;
}