#include "opencv2/highgui/highgui.hpp"
#include "cv.hpp"
#include <bitset>
#include <iostream>
#include <cctype>
#include <stdio.h>
using namespace cv;
using namespace std;

#define RESOLUTION_X 320
#define RESOLUTION_Y 240
inline string num2str(int i)
{
        stringstream ss;
        ss<<i;
        return ss.str()+".png";
}

int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "ERROR: Cannot open the video file" << endl;
        return -1;
    }

   bitset<1> flag(0);
   namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

   cout << "Frame Size = " << dWidth << "x" << dHeight << endl;

   Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));

   //VideoWriter oVideoWriter ("D:/MyVideo.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true); //initialize the VideoWriter object 
   // if ( !oVideoWriter.isOpened() ) //if not initialize the VideoWriter successfully, exit the program
   //{
   //     cout << "ERROR: Failed to write the video" << endl;
   //     return -1;
   //}


    string filename;
    int i=0;

    while (1)
    {

        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
             cout << "ERROR: Cannot read a frame from video file" << endl;
             break;
        }

         //oVideoWriter.write(frame); //writer the frame into the file

        resize(frame,frame,cvSize(RESOLUTION_X,RESOLUTION_Y));

        imshow("MyVideo", frame); //show the frame in "MyVideo" window

        if(flag.any())
        {
            filename=num2str(i++);
            imwrite(filename,frame);
            cout<<filename<<" saved!" << endl;
        }

        int k=waitKey(80);
        switch (k)
        {
            case 's':
                flag.flip();
                continue;
            case 27:
                cout << "esc key is pressed by user" << endl;
                return 0;
        }
    }
    return 0;
}


