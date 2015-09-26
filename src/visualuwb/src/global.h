/*
 * global.h
 *
 *  Created on: Sep 26, 2015
 *      Author: jeffsan
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <iostream>
using namespace std;
#define PI 3.141592654
extern const int N=3;
int ID;
class NetPack
{
public:
	double    dirNection[N+1][2];
	double    decision[N+1][2];//前进，转弯
	friend ostream & operator<< (ostream & h,NetPack & pack)
	{
			cout<<"decision"<<endl;
			for(int i=0;i<N+1;i++)
				cout <<pack.dirNection[i][0]<<" "<<pack.dirNection[i][1]<<"     "
				<<pack.decision[i][0]<<" "<<pack.decision[i][1]<<endl;
			return h;
	}
};

typedef NetPack netpack;
typedef NetPack Netpack;

class Rawinfo
{
public:
	double position[N+1][2];//0：红 1：橙 2：青 3：兰
	double direction[N+1][2];
	Rawinfo();
	friend ostream & operator<< (ostream & h,Rawinfo & info)
	{
		cout<<"position and orientation"<<endl;
		for(int i=0;i<N+1;i++)
			cout <<info.position[i][0]<<" "<<info.position[i][1]<<"     "
			<<info.direction[i][0]<<" "<<info.direction[i][1]<<endl;
		return h;
	}
};
Rawinfo::Rawinfo()
{
	memset(position, 0, sizeof(position));
	memset(direction, 0, sizeof(direction));
}
typedef Rawinfo rawInfo;
typedef Rawinfo RawInfo;
typedef Rawinfo rawinfo;

Netpack pack_test,pack_stop;
rawinfo info_test;
void inite()
{
	pack_test.decision[1][0]=1;
	pack_test.decision[1][1]=300;
	pack_test.decision[2][0]=1;
	pack_test.decision[2][1]=300;
	pack_test.decision[3][0]=-1;
	pack_test.decision[3][1]=300;

	memset(&pack_stop,0,sizeof(pack_stop));

	info_test.position[0][0]=0;
	info_test.position[0][1]=0;
	info_test.position[1][0]=-1;
	info_test.position[1][1]=0;
	info_test.position[2][0]=-1;
	info_test.position[2][1]=-1;
	info_test.position[3][0]=-1;
	info_test.position[3][1]=1;

	info_test.direction[0][0]=1;
	info_test.direction[0][1]=0;
	info_test.direction[1][0]=1;
	info_test.direction[1][1]=0;
	info_test.direction[2][0]=1;
	info_test.direction[2][1]=0;
	info_test.direction[3][0]=1;
	info_test.direction[3][1]=0;
}

#endif /* GLOBAL_H_ */
