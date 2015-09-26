/*
 * rendevzous.cpp
 *
 *  Created on: Sep 26, 2015
 *      Author: jeffsan
 */

#include "global.h"
#include "iostream"
#include <math.h>
#include <map>
using namespace std;
#define ROBOT_ID         1//根据不同机器人修改
#define PI               3.1415926
#define K                1.5//全局调节因子
#define M                4.0//包围调节因子，应为整数
#define N                3//机器人数量
#define MAX_SPEED        12//机器人最大 速度
#define LOGN2            0.628//log(N,2)
#define STEP             0.2//处理两帧视频的时间间隔
#define TAGET_SPEED      15.0//目标最大速度
#define DIS_MIN          60
struct ss_Pack
{
	double dis[N+1];//ID:自己
	double friend_angle[N+1][2];//0左（顺），1右（逆）
	double hunting_angle[N+1][2];//0左，1右
	double target_angle[N+1];//对ZIGBEE要求:目标在右前方返回正值，左前方负值
};

ss_Pack a[3];
class Hunting
{
public:
	Hunting();
	double Getangle(double ,double ,double ,double ,double );
private:
	double lamda;//两个最快方向夹角
	double deta;//包围影响因子
	double gamma;//捕获影响因子
	double k;//全局调节因子
	double m;//包围调节因子，应为整数
	double decision;//相对于目标的决策角，不是最终返回值
};

Hunting::Hunting()
{
	lamda = 0;
	deta  = 0;
	gamma = 0;
	k     = K;
	m     = M;
	decision = 0;
}

double Hunting::Getangle(double clock,double unclock,double dis,double dis_sum,double maxspeed)
{
	//C++不能计算负数的指数，此处同时记录转角方向（左右）
	int flag=0;
	if(clock-unclock<0)//顺时针方向
	{
		deta  = pow(-(clock-unclock)/(2*PI),1.0/m);
		flag=1;
	}
	else //逆时针
	{
		deta  = pow((clock-unclock)/(2*PI),1.0/m);
		flag=-1;
	}

	gamma    = sin(PI*pow(1.0*dis/dis_sum,LOGN2));
	lamda    = 4.0/9*PI;//acos(maxspeed*STEP/dis);

	decision = lamda*(1-pow(2.71828,-1.0*k*deta*gamma));
	return flag*decision;
}


class Robot:Hunting
{
public:
	NetPack  MakeDeci(Rawinfo);//唯一外部接口,返回转角，右转为正，左转为负
	Robot();
	bool captured;
protected:
	NetPack pack;
	ss_Pack ss_pack;//1 2 3三台车
	Hunting A,B,C;
	ss_Pack Calculate(double pos[][2],double dir[][2]);//内部信息接口
	double  maxspeed;
	double AbsoluteAngle(int x,int y);
	double VecAngle(double a[2],double b[2]);
	void PointMinus(double dest[2],double a[2],double b[2]);
	double EuclidNorm(double a[2],double b[2]);
	char* flag_capture;
	//CvPoint position_capture;
	//CvFont font_capture;
};

Robot::Robot()
{
	maxspeed      = MAX_SPEED;
	captured=false;
	flag_capture="Captured";
	//cvInitFont(&font_capture,CV_FONT_HERSHEY_DUPLEX,1.0,1.0,0.0,2,8);
}

class myFilter{
public:
	myFilter():state(false){}
	bool operator() (int in,int dis_low,int dis_high){
		if (state && in>dis_high) state=false;
		if (!state && in<dis_low) state=true;
		return state;
	}
	bool last_one() {return state;}
	void set_false() {state=false;}
private:
	bool state;
}DisMinFilter[4];

NetPack Robot::MakeDeci(Rawinfo rawinfo)
{
	int simple=0;
	ss_pack=Calculate(rawinfo.position, rawinfo.direction);//获得当前全局信息

	double dis_sum=ss_pack.dis[1]+ss_pack.dis[2]+ss_pack.dis[3];
	double angle,deci;
	for(int i=1;i<=N;i++)
	{
		deci=Getangle(
		ss_pack.friend_angle[i][0],ss_pack.friend_angle[i][1],
		ss_pack.dis[i],dis_sum,maxspeed);

		angle=ss_pack.target_angle[i]+deci;

		pack.decision[i][1] = -angle/PI*180;
		pack.decision[i][0] = 1;

		DisMinFilter[i](ss_pack.dis[i],DIS_MIN,DIS_MIN+2);

		if(rawinfo.position[0][0]==0||DisMinFilter[i].last_one()
		 ||rawinfo.position[i][0]==0||rawinfo.position[i][1]==0)
		 //||rawinfo.direction[i][0]==0||rawinfo.direction[i][1]==0){
		{
			pack.decision[i][0]=0;
			if(!(rawinfo.position[0][0]!=0 && rawinfo.position[0][1]!=0
			   && rawinfo.position[i][0]!=0))
			   DisMinFilter[i].set_false();
		}
	}

	captured=DisMinFilter[1].last_one()||DisMinFilter[2].last_one()||DisMinFilter[3].last_one(); //if any three is captured.
	if (captured){
		if(rawinfo.position[0][0]<=320 && rawinfo.position[0][1]<=240)
		{
			//position_capture.x=400;position_capture.y=70;
			//cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
		}
		else if(rawinfo.position[0][0]<=320 && rawinfo.position[0][1]>=240)
		{
			//position_capture.x=400;position_capture.y=410;
			//cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
		}
		else if(rawinfo.position[0][0]>=320 && rawinfo.position[0][1]<=240)
		{
			//position_capture.x=70;position_capture.y=70;
			//cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
		}
		else if(rawinfo.position[0][0]>=320 && rawinfo.position[0][1]>=240)
		{
			//position_capture.x=70;position_capture.y=410;
			//cvPutText(contours,flag_capture,position_capture,&font_capture,CV_RGB(0,0,0));
		}
		//cvShowImage("Global",contours);
	}
	return pack;

}

ss_Pack Robot::Calculate(double pos[][2],double dir[][2])
{
	int i,j;
	ss_Pack res;
	res.dis[0]=0;	//useless...
	for (i=1;i<=N;i++)
		res.dis[i]=EuclidNorm(pos[i],pos[0]);
	res.target_angle[0]=0;
	res.friend_angle[0][0]=0;
	res.friend_angle[0][1]=0;
	//Haunting Angle
	for (i=0;i<=N;i++)
	{
		res.hunting_angle[i][0]=2*PI/3;
		res.hunting_angle[i][1]=2*PI/3;
	}
	//Target Angle
	for (i=1;i<=N;i++){
		double tpoint[2];double tmp;
		PointMinus(tpoint,pos[0],pos[i]);	//tpoint=pos.tar-pos.me
		tmp=-VecAngle(tpoint,dir[i]);
		if (i==2&&0){
			cout<<"i="<<i<<"dir="<<dir[i][0]<<","<<dir[i][1];
			cout<<"\tTar="<<pos[0][0]<<","<<pos[0][1]<<endl;
			cout<<"\tTpoint="<<tpoint[0]<<","<<tpoint[1]<<"\tang="<<tmp<<endl;
		}
		res.target_angle[i]=tmp;
	}
	//Friend Angle, based on target
	double tbase[2];
	multimap<double,int> FriendMap;
	PointMinus(tbase,pos[1],pos[0]);	//base vec:me0->tar
	FriendMap.insert(std::pair<double,int>(0,1));
	for (i=2;i<=N;i++){
		double tpoint[2];double ang;
		PointMinus(tpoint, pos[i], pos[0]);
		ang=VecAngle(tpoint,tbase);		//tpoint.Angle-tbase.Angle,anticounter angle from tbase to tpoint
		FriendMap.insert(std::pair<double,int>(ang,i));
	}
	multimap<double,int>::iterator iter,iter2;
	//show FriendMap
	/*for (iter=FriendMap.begin();iter!=FriendMap.end();iter++)
		cout<<"id="<<iter->second<<",ang="<<iter->first<<endl;*/
	iter=FriendMap.begin();iter2=iter;iter2++;
	double val;
	for (;iter2!=FriendMap.end();){
		val=iter2->first-iter->first;
		if (val<0) throw;
		res.friend_angle[iter->second][1]=val;
		res.friend_angle[iter2->second][0]=val;
		iter++;iter2++;
	}
	iter2=FriendMap.begin();
	val=iter2->first-iter->first;
	if (val<0) val+=2*PI;
	res.friend_angle[iter->second][1]=val;	//last one's right
	res.friend_angle[iter2->second][0]=val;	//first one's left
	return res;
}

double Robot::AbsoluteAngle(int x,int y){
	static double ang;
	if (y==0) return x>0?0:PI;
	if (x==0) return y>0?(PI/2):(-PI/2);
	ang=atan(y/(float)x);
	if (ang<0) ang+=2*PI;
	ang=x>0?ang:(ang+PI);
	if (ang>PI) return ang-2*PI;
	if (ang<-PI) return ang+2*PI;
	return ang;
}

double Robot::VecAngle(double a[2],double b[2]){
	double angA=AbsoluteAngle(a[0],a[1]);
	double angB=AbsoluteAngle(b[0],b[1]);
	angB=angA-angB;
	if (angB>PI) return angB-2*PI;
	if (angB<-PI) return angB+2*PI;
	return angB;
}

void Robot::PointMinus(double dest[2],double a[2],double b[2]){
	dest[0]=a[0]-b[0];
	dest[1]=a[1]-b[1];
}

double Robot::EuclidNorm(double a[2],double b[2]){
	double dx=a[0]-b[0];
	double dy=a[1]-b[1];
	dx*=dx;dy*=dy;
	return sqrt(dx+dy);
}

int main()
{
	Robot   robot;
	rawinfo  info;
	NetPack  pack;
	inite();

	pack=robot.MakeDeci(info_test);

	cout<<info_test;
	cout<<pack;


	return 0;
}
