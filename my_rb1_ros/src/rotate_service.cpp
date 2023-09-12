#include "ros/forwards.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cmath>

float cur_degree = 0;

void od_callback(const nav_msgs::Odometry::ConstPtr &msg){
    float x = msg->pose.pose.orientation.z;
    float y = msg->pose.pose.orientation.y;
    float z = msg->pose.pose.orientation.z;
    float w = msg->pose.pose.orientation.w;

    cur_degree = atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
}

bool rt_callback(my_rb1_ros::Rotate::Request &req,
                 my_rb1_ros::Rotate::Response &rep) {
                 
    float exp_dg = req.degrees/180.0*M_PI + cur_degree;
    float dev_dg = exp_dg-cur_degree;

    ROS_INFO("DEV is %f",dev_dg);
    ros::Rate r(5);
    ros::NodeHandle nh1;
    ros::Publisher vel_pub;
    geometry_msgs::Twist turn_vl;
    while(ros::ok()){
        turn_vl.angular.z = dev_dg/4.0;
        // ROS_INFO("CUR is %f",cur_degree); // checking the data is correct
        // ROS_INFO("EXP is %f",exp_dg);
        // ROS_INFO("DEV is %f",dev_dg);
        vel_pub = nh1.advertise<geometry_msgs::Twist>("cmd_vel",1000);
        vel_pub.publish(turn_vl);
        ros::spinOnce();
        r.sleep();
        dev_dg = cur_degree - exp_dg;
        if (fabs(dev_dg) < 0.003) break;
    }
    turn_vl.angular.z =0.0;
    vel_pub.publish(turn_vl);
    rep.result = "Completely Success";

    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Control_RB1");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("odom",1000, od_callback);
  ros::ServiceServer service =
      nh.advertiseService("/rotate_robot", rt_callback);

  ros::spin();
  return 0;

}