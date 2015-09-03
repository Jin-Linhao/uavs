#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "ssuwb.h"
#include "visualuwb/uwb.h"

int main(int argc, char *argv[])
{
    //int dis;
    int nodes[4];
    nodes[0]=303;
    nodes[1]=303;
    nodes[2]=303;
    nodes[3]=303;
    double x[4]={1,2,3,4};
    double y[4]={1,2,3,4};
    double z[4]={1,2,3,4};

    ros::init(argc, argv, "uwb");
    ros::NodeHandle n;

    if (argc!=3)
    {
        printf("error usage: demo /dev/ttyACM0 serial/usb\n");
        exit(0);
    }

    ssUWB uwb(argv[1],argv[2]);

    ros::Publisher uwb_pub = n.advertise<visualuwb::uwb>("uwb_dis", 1);

    visualuwb::uwb msg;
    msg.header.frame_id = "map";

    while(ros::ok())
    {
        for (int i=0; i<4; i++)
        {
            msg.dis ==uwb.uwb(nodes[i]);
            msg.header.stamp = ros::Time::now();
            msg.anchor.x = x[i];
            msg.anchor.y = y[i];
            msg.anchor.z = z[i];
            uwb_pub.publish(msg);
            ROS_INFO("ID:%d,dis:%d,anchor:[%.2f,%.2f,%.2f]\n",nodes[i],msg.dis,x[i],y[i],z[i]);
        }

    }
    return 0;
}


