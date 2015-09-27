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
using namespace std;
bool server(visualuwb::Rendezvous::Request  &req,
         visualuwb::Rendezvous::Response &res)
{
    Robot   robot;
    rawinfo  info;
    NetPack  pack;
    geometry_msgs::Twist twist;
    for (int i = 0; i <=NumberofRobots; i++ )
    {
        info.position[i][0] = req.pose[i].position.x;
        info.position[i][1] = req.pose[i].position.y;
    }

    pack=robot.MakeDeci(info);
    cout<<info;
    cout<<pack;

    for (int i = 0; i <=NumberofRobots; i++ )
    {
        res.twist.push_back(twist);// [0].linear.x=pack.decision[i][0];
        //res.twist[0].linear.y=pack.decision[i][1];
    }

    //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    //ROS_INFO("sending back response: [%ld]", (long int)res.sum);

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
