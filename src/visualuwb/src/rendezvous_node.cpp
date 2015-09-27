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
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_server");
    Robot   robot;
    rawinfo  info;
    NetPack  pack;
    init_testpack();

    pack=robot.MakeDeci(info_test);
    cout<<info_test;
    cout<<pack;

    return 0;
}
