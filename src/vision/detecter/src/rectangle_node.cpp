#include "rectangle/RectangleDetector.h"
#include "rectangle/OpenCVTimer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <detecter/RectangleTarget.h>
#include <geometry_msgs/Point.h>
#include "sslib.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;
sstimer timer;
image_transport::Publisher* ppub;
ros::Publisher*             ppb;
vector<vector<cv::Point> > squares;
vector<vector<cv::Point> > colorSquares[3];
cv::Mat frame,result;
CRectangleDetector detector;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

        frame=cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Mat image=frame;

        if( frame.empty() )
        {
            cout << "Couldn't load frame!!" << endl;
            return ;
        }


        detector.FindSquares(frame, squares, colorSquares);
        result=detector.DrawSquares(frame, squares);


//        for( size_t i = 0; i < squares.size(); i++ )
//        {
//            const cv::Point* p = &squares[i][0];
//            int n = (int)squares[i].size();
//            polylines(image, &p, &n, 1, true, cv::Scalar(0,255,255), 3, CV_AA);
//        }



        detecter::RectangleTarget target;
       // target.tar.x=loc.xaxis;
       // target.tar.y=loc.yaxis;
        target.image_height = frame.size().height;
        target.image_width  = frame.size().width;
       // if(target.tar[0].x!=0&&target.tar[0].y!=0)
            ppb->publish(target);





        sensor_msgs::ImagePtr imagemsg;
        imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
        ppub->publish(imagemsg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);

    ros::NodeHandle nh;
    image_transport::ImageTransport itt(nh);
    image_transport::Publisher pub = itt.advertise("/detecter/rectangle_result_image", 1);
    ppub=&pub;

    ros::NodeHandle nhp;
    ros::Publisher pb = nhp.advertise<detecter::RectangleTarget>("/detecter/rectangle_target_point", 1);
    ppb=&pb;

    ros::spin();
    cv::destroyWindow("Output");
    return 0;
}
