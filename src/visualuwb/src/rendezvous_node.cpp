#include "rendezvous.h"

int main()
{
    Robot   robot;
    rawinfo  info;
    NetPack  pack;
    init_testpack();

    pack=robot.MakeDeci(info_test);
    cout<<info_test;
    cout<<pack;

    return 0;
}
