#include "ros/ros.h"
#include "tracker/targets.h"
#include "../include/tracker/MultiObjectTLD/camExample.h"
#include <math.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<tracker::targets>("targets", 2);
    Init(argc, argv);
    vector<ssBox> boxes;
    tracker::targets targets;
    sensor_msgs::RegionOfInterest roi;
    targets.image_height=240;
    targets.image_width=320;
    targets.focal_lenth_x=100;
    targets.focal_lenth_y=100;

    while(ros::ok())
    {
        boxes=Run();

        for(vector<ssBox>::const_iterator iter=boxes.begin(); iter!=boxes.end(); ++iter)
        {
            ROS_INFO("TARGETS:%d,%d,%d,%d",(*iter).x,(*iter).y,(*iter).width,(*iter).height);
            roi.x_offset   = (*iter).x;
            roi.y_offset   = (*iter).y;
            roi.height     = (*iter).height;
            roi.width      = (*iter).width;
            roi.do_rectify = 1;
            targets.roi.push_back(roi);
        }
        if(boxes.size()!=0)
        {
            pub.publish(targets);
            targets.roi.clear();
        }
    }

    cvReleaseCapture(&capture);
    cvDestroyAllWindows();
    return 0;
}
