#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include "cv.hpp"
#include <bitset>
#include <iostream>
#include <cctype>
#include <stdio.h>
using namespace cv;
using namespace std;

#define RESOLUTION_X 640//320
#define RESOLUTION_Y 480//240
string database;
int i=0;
bitset<1> flag(0);

inline string num2str(int i)
{
        stringstream ss;
        ss<<i;
        return database+ss.str()+".png";
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        string filename;

        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        resize(frame,frame,cvSize(RESOLUTION_X,RESOLUTION_Y));

        //if(flag.any())
        //{
            filename=num2str(i++);
            imwrite(filename,frame);
            ROS_INFO("%s saved!!",filename.c_str());
        //}

        int k=waitKey(80);
        switch (k)
        {
            case 's':
                flag.flip();
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "recorder");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    nh.getParam("/recorder/database",database);
    ros::spin();
    return 0;
}


