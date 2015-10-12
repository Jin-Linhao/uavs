#include "camExample.h"
int main(int argc, char *argv[])
{
    Init(argc, argv);
    vector<ssBox> boxes;
    while(1)
    {
        boxes=Run();
        for(vector<ssBox>::const_iterator iter=boxes.begin(); iter!=boxes.end(); ++iter)
            cout<<(*iter).x<<" "<<(*iter).y<<' '<<(*iter).width<<' '<<(*iter).height<<endl;
    }

    cvReleaseCapture(&capture);
    cvDestroyAllWindows();
    return 0;
}
