#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "circle/EllipseDetectorLX.h"
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <sys/time.h>
#include <cv_bridge/cv_bridge.h>
#include <detecter/CircleTarget.h>
#include "sslib.h"
using namespace std;
using namespace cv;
CEllipseDetectorLX yaed;
location loc;
VideoCapture capp=yaed.opencamera();
sstimer timer;
image_transport::Publisher* ppub;
ros::Publisher*             ppb;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        Mat3b image  = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat3b result = yaed.Onvideotracking(image);
        loc=yaed.locations();
        //ROS_INFO("location: %d,%d",loc.xaxis,loc.yaxis);

        //geometry_msgs::Point point;
        detecter::CircleTarget target;
        target.tar.x=loc.xaxis;
        target.tar.y=loc.yaxis;
        target.image_height = image.size().height;
        target.image_width  = image.size().width;
        if(target.tar.x!=0&&target.tar.y!=0)
            ppb->publish(target);


        sensor_msgs::ImagePtr imagemsg;
        imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
        ppub->publish(imagemsg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    ros::NodeHandle nh;
    image_transport::ImageTransport itt(nh);
    image_transport::Publisher pub = itt.advertise("/detecter/circle_result_image", 1);
    ppub=&pub;

    ros::NodeHandle nhp;
    ros::Publisher pb = nhp.advertise<detecter::CircleTarget>("/detecter/circle_target_point", 1);
    ppb=&pb;

    ros::spin();
    cv::destroyWindow("Output");
	return 0;	   
}
