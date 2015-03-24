#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>
using namespace std;
string database;


inline string num2str(int i)
{
        stringstream ss;
        ss<<i;
        return database+ss.str()+".png";
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    nh.getParam("/images/database",database);

    int hz=15;

    if (!nh.getParam("/video/hz", hz))
        ROS_WARN("Get parameter /video/hz fail...");

    ROS_INFO("pushlish image in %d hz",hz);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(hz);

    int i=0;

    string filename;

    while (nh.ok())
    {  
        filename=num2str(i++);
        frame=cv::imread(filename);

        if(frame.data)
            ROS_INFO("Publish images %s",filename.c_str());
        else
            ROS_ERROR("Can not read imges!!");

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
