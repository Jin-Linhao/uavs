#include <iostream>
#include <sys/time.h>
using namespace std;

class sstimer
{
public:
    inline void start()
	{
        gettimeofday(&starttime,0);
	}
    double end()
    {
        gettimeofday(&endtime,0);
        timeuse = 1000000*(endtime.tv_sec - starttime.tv_sec) + endtime.tv_usec - starttime.tv_usec;
        timeuse /=1000;
        cout<<"Frequency: "<<1000/timeuse<<endl;
        return timeuse;
    }
private:
    timeval starttime,endtime;
    double timeuse;
};


