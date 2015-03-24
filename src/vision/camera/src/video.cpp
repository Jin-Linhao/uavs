#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv,"image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);


    int vport=0;
    int hz=15;
    if (!nh.getParam("/video/videoport", vport))
        ROS_WARN("Get parameter /video/videoport fail...");
    if (!nh.getParam("/video/hz", hz))
        ROS_WARN("Get parameter /video/hz fail...");

    ROS_INFO("Connecting to /dev/video%d\n....",vport);
    ROS_INFO("pushlish image in %d hz",hz);
    cv::VideoCapture cap(vport);
    if(!cap.isOpened())  // check if we succeeded
        ROS_ERROR("Open camera /dev/video%d error!",vport);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(hz);

    while (nh.ok())
    {
        cap >> frame; // get a new frame from camera
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
