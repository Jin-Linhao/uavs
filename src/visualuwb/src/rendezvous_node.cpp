/*
 * rendezvous_node.cpp
 *
 *  Created on: Sep 26, 2015
 *      Author: jeffsan
 */
#include <iostream>
#include "rendezvous.h"
#include "ros/ros.h"
#include "visualuwb/Rendezvous.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
using namespace std;

bool server(visualuwb::Rendezvous::Request  &req,
         visualuwb::Rendezvous::Response &res)
{
    Robot   robot;
    rawinfo  info;
    NetPack  pack;
    ROS_INFO("new requirement!");

    for (int i = 0; i <=NumberofRobots; i++ )
    {
        info.position[i][0] = req.pose[i].position.x;
        info.position[i][1] = req.pose[i].position.y;
        tf::Quaternion q(req.pose[i].orientation.x,
                         req.pose[i].orientation.y,
                         req.pose[i].orientation.z,
                         req.pose[i].orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
        info.direction[i][0]=cos(yaw);
        info.direction[i][1]=sin(yaw);
    }

    cout<<info;
    pack=robot.MakeDeci(info);
    cout<<pack<<endl;

    for (int i = 0; i <=NumberofRobots; i++ )
    {
        geometry_msgs::Twist twist;
        twist.linear.x = sin(pack.decision[i][1]/180.0*PI);
        twist.linear.y = cos(pack.decision[i][1]/180.0*PI);
        double length = sqrt(twist.linear.x*twist.linear.x+ twist.linear.y*twist.linear.y);
        twist.linear.x /=length;
        twist.linear.y /= length;
        twist.linear.z = 0.2;
        res.twist.push_back(twist);
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rendezvous_server");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("rendezvous_service", server);

    ROS_INFO("Ready to rendezvous control.");

    ros::spin();

    return 0;
}
