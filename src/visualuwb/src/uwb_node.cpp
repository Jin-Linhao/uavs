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
    nodes[0]=101;
    nodes[1]=102;
    nodes[2]=105;
    nodes[3]=106;
    double x[4]={-3.0, 3.0, 3.0,-3.0};
    double y[4]={-3.0,-3.0, 3.0, 3.0};
    double z[4]={-1.78,-1.17,-1.31,-1.31};

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
            msg.dis =uwb.uwb(nodes[i]);
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


