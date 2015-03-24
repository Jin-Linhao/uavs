#include <opencv2/opencv.hpp>
#include "EllipseDetectorLX.h"
#include <fstream>
#include <iostream>
#include <iomanip>

location locationn;
    char result[64];
    locationdis locationdisp;


CEllipseDetectorLX::CEllipseDetectorLX(void)
{
	// Default Parameters Settings
	_szPreProcessingGaussKernelSize	= Size(5,5);
	_dPreProcessingGaussSigma		= 1.0;
	_fThPosition					= 1.0f;
	_fMaxCenterDistance				= 100.0f * 0.05f;
	_fMaxCenterDistance2			= _fMaxCenterDistance * _fMaxCenterDistance;
	_iMinEdgeLength					= 16;
	_fMinOrientedRectSide			= 3.0f;
	_fDistanceToEllipseContour		= 0.1f;
	_fMinScore						= 0.4f;
	_fMinReliability				= 0.4f;
	_iNs							= 16;

	 srand ( unsigned(time(NULL)) );	
}


CEllipseDetectorLX::~CEllipseDetectorLX(void)
{	
}

void CEllipseDetectorLX::SetParameters	(	Size	szPreProcessingGaussKernelSize,
												double	dPreProcessingGaussSigma,
												double 	fThPosition,
												float	fMaxCenterDistance,
												int		iMinEdgeLength,
												float	fMinOrientedRectSide,
												float	fDistanceToEllipseContour,
												float	fMinScore,
												float	fMinReliability,
												int     iNs
											)
{
	_szPreProcessingGaussKernelSize		= szPreProcessingGaussKernelSize;
	_dPreProcessingGaussSigma			= dPreProcessingGaussSigma;
	_fThPosition						= fThPosition;	
	_fMaxCenterDistance					= fMaxCenterDistance;	
	_iMinEdgeLength						= iMinEdgeLength;
	_fMinOrientedRectSide				= fMinOrientedRectSide;
	_fDistanceToEllipseContour			= fDistanceToEllipseContour;
	_fMinScore							= fMinScore;
	_fMinReliability					= fMinReliability;
	_iNs								= iNs;
	
	_fMaxCenterDistance2				= _fMaxCenterDistance * _fMaxCenterDistance;

}

uint inline CEllipseDetectorLX::GenerateKey(uchar pair, ushort u, ushort v)
{
	return (pair << 30) + (u << 15) + v;
};



int CEllipseDetectorLX::FindMaxK(const int* v) const
{
	int max_val = 0;
	int max_idx = 0;
	for(unsigned i=0; i<ACC_K_SIZE; ++i)
	{
		(v[i] > max_val) ? max_val = v[i], max_idx = i : NULL;
	}
	
	return max_idx + 90;
};

int CEllipseDetectorLX::FindMaxN(const int* v) const
{
	int max_val = 0;
	int max_idx = 0;
	for(unsigned i=0; i<ACC_N_SIZE; ++i)
	{
		(v[i] > max_val) ? max_val = v[i], max_idx = i : NULL;		
	}
	
	return max_idx;
};

int CEllipseDetectorLX::FindMaxA(const int* v) const
{
	int max_val = 0;
	int max_idx = 0;
	for(unsigned i=0; i<ACC_A_SIZE; ++i)
	{
		(v[i] > max_val) ? max_val = v[i], max_idx = i : NULL;		
	}
	
	return max_idx;
};


float CEllipseDetectorLX::GetMedianSlope(vector<Point2f>& med, Point2f& M, vector<float>& slopes)
{
	//med	: vector of points
	//M		: centroid of the points in med

	unsigned iNofPoints = med.size();
	//CV_Assert(iNofPoints >= 2);

	unsigned halfSize = iNofPoints >> 1;
	unsigned quarterSize = halfSize >> 1;

	vector<float> xx, yy;
	slopes.reserve(halfSize);
	xx.reserve(iNofPoints);
	yy.reserve(iNofPoints);

	for(unsigned i=0; i<halfSize; ++i)
	{
		Point2f& p1 = med[i];
		Point2f& p2 = med[halfSize + i];

		xx.push_back(p1.x);
		xx.push_back(p2.x);
		yy.push_back(p1.y);
		yy.push_back(p2.y);

		float den = (p2.x - p1.x);
		float num = (p2.y - p1.y);

		if(den == 0) den = 0.00001f;
				
		slopes.push_back(num / den);
	}

	nth_element(slopes.begin(), slopes.begin() + quarterSize, slopes.end());
	nth_element(xx.begin(), xx.begin() + halfSize, xx.end());
	nth_element(yy.begin(), yy.begin() + halfSize, yy.end());
	M.x = xx[halfSize];
	M.y = yy[halfSize];

	return slopes[quarterSize];
};




void CEllipseDetectorLX::GetFastCenter	(	vector<Point>& e1,
												vector<Point>& e2,
												EllipseData& data
											)
{
	data.isValid = true;

	uint size_1 = e1.size();
	uint size_2 = e2.size();

	uint hsize_1 = size_1 >> 1;
	uint hsize_2 = size_2 >> 1;

	Point& med1 = e1[hsize_1];
	Point& med2 = e2[hsize_2];

	Point2f M12,M34;
	float q2, q4;

	{
		//first to second

		//reference slope
		
		float dx_ref =  e1[0].x - med2.x;
		float dy_ref =  e1[0].y - med2.y;
		if(dy_ref == 0) dy_ref = 0.00001f;

		float m_ref = dy_ref / dx_ref;
		data.m1 = m_ref;

		//find points with same slope as reference
		vector<Point2f> med;
		med.reserve(hsize_2);

		uint minPoints = (_iNs < hsize_2) ? _iNs : hsize_2;

		vector<uint> indexes(minPoints);
		if(_iNs < hsize_2)
		{
			unsigned iSzBin = hsize_2 / unsigned(_iNs);
			unsigned iIdx = hsize_2 + (iSzBin / 2);

			for(int i=0; i<_iNs; ++i)
			{
				indexes[i] =  iIdx;
				iIdx += iSzBin;
			}
		}
		else
		{
			iota(indexes.begin(), indexes.end(), hsize_2);
		}

		for(uint ii=0; ii<minPoints; ++ii)
		{
			uint i = indexes[ii];

			float x1 = float(e2[i].x);
			float y1 = float(e2[i].y);

			uint begin = 0;
			uint end   = size_1-1;

			float xb = float(e1[begin].x);
			float yb = float(e1[begin].y);
			float res_begin = ((xb - x1) * dy_ref) - ((yb - y1) * dx_ref);
			int sign_begin = sgn(res_begin);
			if(sign_begin == 0)
			{
				//found
				med.push_back(Point2f((xb+x1)* 0.5f, (yb+y1)* 0.5f));
				continue;
			}

			float xe = float(e1[end].x);
			float ye = float(e1[end].y);
			float res_end = ((xe - x1) * dy_ref) - ((ye - y1) * dx_ref);
			int sign_end = sgn(res_end);
			if(sign_end == 0)
			{
				//found
				med.push_back(Point2f((xe+x1)* 0.5f, (ye+y1)* 0.5f));
				continue;
			}

			if((sign_begin + sign_end) != 0)
			{
				continue;
			}

			uint j = (begin + end) >> 1;

			while(end - begin > 2)
			{
				float x2 = float(e1[j].x);
				float y2 = float(e1[j].y);
				float res = ((x2 - x1) * dy_ref) - ((y2 - y1) * dx_ref);
				int sign_res = sgn(res);

				if(sign_res == 0)
				{
					//found
					med.push_back(Point2f((x2+x1)* 0.5f, (y2+y1)* 0.5f));
					break;
				}

				if(sign_res + sign_begin == 0)
				{
					sign_end = sign_res;
					end = j;
				}
				else
				{
					sign_begin = sign_res;
					begin = j;
				}
				j = (begin + end) >> 1;
			}

			med.push_back(Point2f((e1[j].x+x1)* 0.5f, (e1[j].y+y1)* 0.5f));
		}

		if(med.size() < 2)
		{
			data.isValid = false;
			return;
		}

		q2 = GetMedianSlope(med, M12, data.slopes1);
	}

	{
		//second to first

		//reference slope
		float dx_ref =  med1.x - e2[0].x;
		float dy_ref =  med1.y - e2[0].y;		
		if(dy_ref == 0) dy_ref = 0.00001f;

		float m_ref = dy_ref / dx_ref;
		data.m2 = m_ref;

		//find points with same slope as reference
		vector<Point2f> med;
		med.reserve(hsize_1);

		uint minPoints = (_iNs < hsize_1) ? _iNs : hsize_1;
		
		vector<uint> indexes(minPoints);
		if(_iNs < hsize_1)
		{
			unsigned iSzBin = hsize_1 / unsigned(_iNs);
			unsigned iIdx = hsize_1 + (iSzBin / 2);

			for(int i=0; i<_iNs; ++i)
			{
				indexes[i] =  iIdx;
				iIdx += iSzBin;
			}
		}
		else
		{
			iota(indexes.begin(), indexes.end(), hsize_1);
		}

		for(uint ii=0; ii<minPoints; ++ii)
		{
			uint i = indexes[ii];

			float x1 = float(e1[i].x);
			float y1 = float(e1[i].y);

			uint begin = 0;
			uint end   = size_2-1;

			float xb = float(e2[begin].x);
			float yb = float(e2[begin].y);
			float res_begin = ((xb - x1) * dy_ref) - ((yb - y1) * dx_ref);
			int sign_begin = sgn(res_begin);
			if(sign_begin == 0)
			{
				//found
				med.push_back(Point2f((xb+x1)* 0.5f, (yb+y1)* 0.5f));
				continue;
			}

			float xe = float(e2[end].x);
			float ye = float(e2[end].y);
			float res_end = ((xe - x1) * dy_ref) - ((ye - y1) * dx_ref);
			int sign_end = sgn(res_end);
			if(sign_end == 0)
			{
				//found
				med.push_back(Point2f((xe+x1)* 0.5f, (ye+y1)* 0.5f));
				continue;
			}

			if((sign_begin + sign_end) != 0)
			{
				continue;
			}

			uint j = (begin + end) >> 1;

			while(end - begin > 2)
			{
				float x2 = float(e2[j].x);
				float y2 = float(e2[j].y);
				float res = ((x2 - x1) * dy_ref) - ((y2 - y1) * dx_ref);
				int sign_res = sgn(res);

				if(sign_res == 0)
				{
					//found
					med.push_back(Point2f((x2+x1)* 0.5f, (y2+y1)* 0.5f));
					break;
				}

				if(sign_res + sign_begin == 0)
				{
					sign_end = sign_res;
					end = j;
				}
				else
				{
					sign_begin = sign_res;
					begin = j;
				}
				j = (begin + end) >> 1;
			}
			med.push_back(Point2f((e2[j].x+x1)* 0.5f, (e2[j].y+y1)* 0.5f));
		}

		if(med.size() < 2)
		{
			data.isValid = false;
			return;
		}
		q4 = GetMedianSlope(med, M34, data.slopes2);
	}

	if(q2 == q4)
	{
		data.isValid = false;
		return;
	}

	float invDen = 1 / (q2 - q4);
	data.center.x = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
	data.center.y = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;	
	data.t1 = q2;
	data.t2 = q4;
	data.M1 = M12;
	data.M2 = M34;
};




void CEllipseDetectorLX::DetectEdges13	(	Mat1b& DP,
												VVP& points_1,
												VVP& points_3
											)
{
	//vector of connected edge points
	VVP contours;

	//labeling 8-connected edge points, discarding edge too small
	Labeling(DP, contours, _iMinEdgeLength);

	int iContoursSize = contours.size();
		
	//for each edge
	for(int i=0; i<iContoursSize; ++i)
	{		
		VP& edgeSegment = contours[i];

		//Selection strategy - Step 1 - See Sect [] of the paper
		//constraint on axes aspect ratio
		RotatedRect oriented = minAreaRect(edgeSegment);
		int o_min = min(oriented.size.width, oriented.size.height);
		if(o_min < _fMinOrientedRectSide)
		{
			continue;
		}

		//order edge points of the same arc
		sort(edgeSegment.begin(), edgeSegment.end(), SortTopLeft2BottomRight);
		int iEdgeSegmentSize = edgeSegment.size();
		
		// get extrema of the arc
		Point& left  = edgeSegment[0];
		Point& right = edgeSegment[iEdgeSegmentSize-1];

		//find convexity - See Sect [] of the paper
		int iCountTop = 0;
		int xx = left.x;
		for(int k = 1; k < iEdgeSegmentSize; ++k)
		{
			if(edgeSegment[k].x == xx) continue;

			iCountTop += (edgeSegment[k].y - left.y);
			xx = edgeSegment[k].x;
		}

		int iCountBottom = ((right.x - left.x) * (right.y - left.y)) - iEdgeSegmentSize - iCountTop;

		if(iCountBottom > iCountTop)
		{	//1
			points_1.push_back(edgeSegment);
		}
		else if (iCountBottom < iCountTop)
		{	//3
			points_3.push_back(edgeSegment);
		}
	}
};


void CEllipseDetectorLX::DetectEdges24	(	Mat1b& DN,
												VVP& points_2,
												VVP& points_4
											)
{
	//vector of connected edge points
	VVP contours;

	///labeling 8-connected edge points, discarding edge too small
	Labeling(DN, contours, _iMinEdgeLength);

	int iContoursSize = contours.size();

	//for each edge
	for(int i=0; i<iContoursSize; ++i)
	{
		VP& edgeSegment = contours[i];
		
		//Selection strategy - Step 1 - See Sect [] of the paper
		//constraint on axes aspect ratio
		RotatedRect oriented = minAreaRect(edgeSegment);
		int o_min = min(oriented.size.width, oriented.size.height);
		if(o_min < _fMinOrientedRectSide)
		{
			continue;
		}

		//order edge points of the same arc
		sort(edgeSegment.begin(), edgeSegment.end(), SortBottomLeft2TopRight);
		int iEdgeSegmentSize = edgeSegment.size();

		// get extrema of the arc
		Point& left  = edgeSegment[0];
		Point& right = edgeSegment[iEdgeSegmentSize-1];

		//find convexity - See Sect [] of the paper
		int iCountBottom = 0;
		int xx = left.x;
		for(int k = 1; k < iEdgeSegmentSize; ++k)
		{
			if(edgeSegment[k].x == xx) continue;

			iCountBottom += (left.y - edgeSegment[k].y);
			xx = edgeSegment[k].x;
		}

		int iCountTop = (abs(right.x - left.x) * abs(right.y - left.y)) - iEdgeSegmentSize - iCountBottom;

		if(iCountBottom > iCountTop)
		{
			//2
			points_2.push_back(edgeSegment);
		}
		else if (iCountBottom < iCountTop)
		{
			//4
			points_4.push_back(edgeSegment);
		}
	}
};

//most important function for detecting ellipses. See Sect[] of the paper
void CEllipseDetectorLX::FindEllipses	(	Point2f& center,
											VP& edge_i,
											VP& edge_j,
											VP& edge_k,
											EllipseData& data_ij,
											EllipseData& data_ik,
											vector<CEllipse>& ellipses
										)
{

	//FIND ELLIPSE PARAMETERS

	// 0-initialize accumulators
	memset(accN, 0, sizeof(int)*ACC_N_SIZE);
	memset(accK, 0, sizeof(int)*ACC_K_SIZE);
	memset(accA, 0, sizeof(int)*ACC_A_SIZE);		

	// get size of the 4 vectors of slopes (2 pairs of arcs)
	int sz_ij1 = int(data_ij.slopes1.size());
	int sz_ij2 = int(data_ij.slopes2.size());
	int sz_ik1 = int(data_ik.slopes1.size());
	int sz_ik2 = int(data_ik.slopes2.size());

	 //get the size of the 3 arcs
	ushort sz_ei = edge_i.size();
	ushort sz_ej = edge_j.size();
	ushort sz_ek = edge_k.size();

	// center of the estimated ellipse
	float a0 = center.x;
	float b0 = center.y;


	// estimation of remaining parameters
	// uses 4 combinations of parameters. See Tab [] and Sect [] in the paper.
	{
		float q1 = data_ij.m1;
		float q3 = data_ik.m1;
		float q5 = data_ik.m2;
		
		for(int ij1=0; ij1<sz_ij1; ++ij1)
		{
			float q2 = data_ij.slopes1[ij1];

			float q1xq2 = q1*q2;

			for(int ik1=0; ik1<sz_ik1; ++ik1)
			{
				float q4 = data_ik.slopes1[ik1];

				float q3xq4 = q3*q4;

				float b = (q3xq4 + 1)*(q1+q2) - (q1xq2 + 1)*(q3+q4);
				float a = (q1xq2 - q3xq4);
				float Kp = (-b + sqrt(b*b + 4*a*a)) / (2*a);
				float zplus = ((q1-Kp)*(q2-Kp)) / ((1+q1*Kp)*(1+q2*Kp));

				if(zplus >= 0.0f)
				{
					continue;
				}

				float Np = sqrt(-zplus);
				float rho = atan(Kp);
				int Kdeg;
				if(Np > 1.f)
				{
					Np = 1.f / Np;
					Kdeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)
					rho += CV_PI * 0.5f;
				}
				else
				{
					Kdeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)
				}

				int iNp = cvRound(Np*100); // [0, 100]

				if	(	0 <= iNp	&& iNp  < ACC_N_SIZE &&
						0 <= Kdeg	&& Kdeg < ACC_K_SIZE
					)
				{
					++ accN[iNp];
					++ accK[Kdeg];
				}
			}

			
			for(int ik2=0; ik2<sz_ik2; ++ik2)
			{
				float q4 = data_ik.slopes2[ik2];

				float q5xq4 = q5*q4;

				float b = (q5xq4 + 1)*(q1+q2) - (q1xq2 + 1)*(q5+q4);
				float a = (q1xq2 - q5xq4);
				float Kp = (-b + sqrt(b*b + 4*a*a)) / (2*a);
				float zplus = ((q1-Kp)*(q2-Kp)) / ((1+q1*Kp)*(1+q2*Kp));

				if(zplus >= 0.0f)
				{
					continue;
				}

				float Np = sqrt(-zplus);
				float rho = atan(Kp);
				int Kdeg;
				if(Np > 1.f)
				{
					Np = 1.f / Np;
					Kdeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)
					rho += CV_PI * 0.5f;
				}
				else
				{
					Kdeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)
				}

				int iNp = cvRound(Np*100); // [0, 100]

				if	(	0 <= iNp	&& iNp  < ACC_N_SIZE &&
						0 <= Kdeg	&& Kdeg < ACC_K_SIZE
					)
				{
					++ accN[iNp];
					++ accK[Kdeg];
				}
			}
			
		}
	}

	
	{
		float q1 = data_ij.m2;
		float q3 = data_ik.m2;
		float q5 = data_ik.m1;

		for(int ij2=0; ij2<sz_ij2; ++ij2)
		{
			float q2 = data_ij.slopes2[ij2];

			float q1xq2 = q1*q2;

			for(int ik2=0; ik2<sz_ik2; ++ik2)
			{
				float q4 = data_ik.slopes2[ik2];

				float q3xq4 = q3*q4;

				float b = (q3xq4 + 1)*(q1+q2) - (q1xq2 + 1)*(q3+q4);
				float a = (q1xq2 - q3xq4);
				float Kp = (-b + sqrt(b*b + 4*a*a)) / (2*a);
				float zplus = ((q1-Kp)*(q2-Kp)) / ((1+q1*Kp)*(1+q2*Kp));

				if(zplus >= 0.0f)
				{
					continue;
				}

				float Np = sqrt(-zplus);
				float rho = atan(Kp);
				int Kdeg;
				if(Np > 1.f)
				{
					Np = 1.f / Np;
					Kdeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)
					rho += CV_PI * 0.5f;
				}
				else
				{
					Kdeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)
				}

				int iNp = cvRound(Np*100); // [0, 100]

				if	(	0 <= iNp	&& iNp  < ACC_N_SIZE &&
						0 <= Kdeg	&& Kdeg < ACC_K_SIZE
					)
				{
					++ accN[iNp];
					++ accK[Kdeg];
				}
			}

			
			for(int ik1=0; ik1<sz_ik1; ++ik1)
			{
				float q4 = data_ik.slopes1[ik1];

				float q5xq4 = q5*q4;

				float b = (q5xq4 + 1)*(q1+q2) - (q1xq2 + 1)*(q5+q4);
				float a = (q1xq2 - q5xq4);
				float Kp = (-b + sqrt(b*b + 4*a*a)) / (2*a);
				float zplus = ((q1-Kp)*(q2-Kp)) / ((1+q1*Kp)*(1+q2*Kp));

				if(zplus >= 0.0f)
				{
					continue;
				}

				float Np = sqrt(-zplus);
				float rho = atan(Kp);
				int Kdeg;
				if(Np > 1.f)
				{
					Np = 1.f / Np;
					Kdeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)
					rho += CV_PI * 0.5f;
				}
				else
				{
					Kdeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)
				}

				int iNp = cvRound(Np*100); // [0, 100]

				if	(	0 <= iNp	&& iNp  < ACC_N_SIZE &&
						0 <= Kdeg	&& Kdeg < ACC_K_SIZE
					)
				{
					++ accN[iNp];
					++ accK[Kdeg];
				}
			}
			
		}
	}

	//find peak in N and K accumulator
	int iN	= FindMaxN(accN);
	int iK	= FindMaxK(accK);

	//recover real values
	float fK = float(iK);
	float Np = float(iN) * 0.01f;
	float rho = fK * CV_PI / 180.f;	//deg 2 rad
	float Kp = tan(rho);

	// estimate A. See Sect [] in the paper

	for(ushort l=0; l<sz_ei; ++l)
	{
		Point& pp = edge_i[l];
		float sk = 1.f / sqrt(Kp*Kp + 1.f);
		float x0 = ((pp.x - a0) * sk) + (((pp.y - b0)*Kp) * sk);
		float y0 = -(((pp.x - a0) * Kp) * sk) + ((pp.y - b0) * sk);
		float Ax = sqrt((x0*x0*Np*Np + y0*y0) / ((Np*Np)*(1.f + Kp*Kp)));
		int A  = cvRound(abs(Ax / cos(rho)));
		if((0 <= A) && (A < ACC_A_SIZE))
		{
			++ accA[A];
		}
	}

	for(ushort l=0; l<sz_ej; ++l)
	{
		Point& pp = edge_j[l];
		float sk = 1.f / sqrt(Kp*Kp + 1.f);
		float x0 = ((pp.x - a0) * sk) + (((pp.y - b0)*Kp) * sk);
		float y0 = -(((pp.x - a0) * Kp) * sk) + ((pp.y - b0) * sk);
		float Ax = sqrt((x0*x0*Np*Np + y0*y0) / ((Np*Np)*(1.f + Kp*Kp)));
		int A  = cvRound(abs(Ax / cos(rho)));
		if((0 <= A) && (A < ACC_A_SIZE))
		{
			++ accA[A];
		}
	}

	for(ushort l=0; l<sz_ek; ++l)
	{
		Point& pp = edge_k[l];
		float sk = 1.f / sqrt(Kp*Kp + 1.f);
		float x0 = ((pp.x - a0) * sk) + (((pp.y - b0)*Kp) * sk);
		float y0 = -(((pp.x - a0) * Kp) * sk) + ((pp.y - b0) * sk);
		float Ax = sqrt((x0*x0*Np*Np + y0*y0) / ((Np*Np)*(1.f + Kp*Kp)));
		int A  = cvRound(abs(Ax / cos(rho)));
		if((0 <= A) && (A < ACC_A_SIZE))
		{
			++ accA[A];
		}
	}

	// find peak in A accumulator
	int A = FindMaxA(accA);
	float fA = float(A);

	// find B value
	float fB = abs(fA * Np);

	// got all ellipse parameters!
	CEllipse ell(a0, b0, fA, fB, fmod(rho, float(CV_PI)));

	// get the score. See Sect [] in the paper

	//find the number of edge pixel lying on the ellipse
	float _cos = cos(-ell._rho);
	float _sin = sin(-ell._rho);

	float invA2 = 1.f / (ell._A * ell._A);
	float invB2 = 1.f / (ell._B * ell._B);

	float invNofPoints = 1.f / float(sz_ei + sz_ej + sz_ek);
	int counter_on_perimeter = 0;

	for(ushort l=0; l<sz_ei; ++l)
	{
		float tx = float(edge_i[l].x) - ell._xc;
		float ty = float(edge_i[l].y) - ell._yc;
		float rx = (tx*_cos - ty*_sin);
		float ry = (tx*_sin + ty*_cos);

		float h = (rx*rx)*invA2 + (ry*ry)*invB2;
		if(abs(h - 1.f) < _fDistanceToEllipseContour)
		{
			++counter_on_perimeter;
		}
	}

	for(ushort l=0; l<sz_ej; ++l)
	{
		float tx = float(edge_j[l].x) - ell._xc;
		float ty = float(edge_j[l].y) - ell._yc;
		float rx = (tx*_cos - ty*_sin);
		float ry = (tx*_sin + ty*_cos);

		float h = (rx*rx)*invA2 + (ry*ry)*invB2;
		if(abs(h - 1.f) < _fDistanceToEllipseContour)
		{
			++counter_on_perimeter;
		}
	}

	for(ushort l=0; l<sz_ek; ++l)
	{
		float tx = float(edge_k[l].x) - ell._xc;
		float ty = float(edge_k[l].y) - ell._yc;
		float rx = (tx*_cos - ty*_sin);
		float ry = (tx*_sin + ty*_cos);

		float h = (rx*rx)*invA2 + (ry*ry)*invB2;
		if(abs(h - 1.f) < _fDistanceToEllipseContour)
		{
			++counter_on_perimeter;
		}
	}

	//no points found on the ellipse
	if(counter_on_perimeter <= 0) return;


	//compute score
	ell._score = float(counter_on_perimeter) * invNofPoints;
	//no enough points found on the ellipse
	if(ell._score < _fMinScore) return;

	//compute reliability	
	float di,dj,dk;
	{
		Point2f p1(float(edge_i[0].x), float(edge_i[0].y));
		Point2f p2(float(edge_i[sz_ei-1].x), float(edge_i[sz_ei-1].y));
		p1.x -= ell._xc;
		p1.y -= ell._yc;
		p2.x -= ell._xc;
		p2.y -= ell._yc;
		Point2f r1((p1.x*_cos - p1.y*_sin),(p1.x*_sin + p1.y*_cos));
		Point2f r2((p2.x*_cos - p2.y*_sin),(p2.x*_sin + p2.y*_cos));
		di = abs(r2.x - r1.x) + abs(r2.y - r1.y);
	}
	{
		Point2f p1(float(edge_j[0].x), float(edge_j[0].y));
		Point2f p2(float(edge_j[sz_ej-1].x), float(edge_j[sz_ej-1].y));
		p1.x -= ell._xc;
		p1.y -= ell._yc;
		p2.x -= ell._xc;
		p2.y -= ell._yc;
		Point2f r1((p1.x*_cos - p1.y*_sin),(p1.x*_sin + p1.y*_cos));
		Point2f r2((p2.x*_cos - p2.y*_sin),(p2.x*_sin + p2.y*_cos));
		dj = abs(r2.x - r1.x) + abs(r2.y - r1.y);
	}
	{
		Point2f p1(float(edge_k[0].x), float(edge_k[0].y));
		Point2f p2(float(edge_k[sz_ek-1].x), float(edge_k[sz_ek-1].y));
		p1.x -= ell._xc;
		p1.y -= ell._yc;
		p2.x -= ell._xc;
		p2.y -= ell._yc;
		Point2f r1((p1.x*_cos - p1.y*_sin),(p1.x*_sin + p1.y*_cos));
		Point2f r2((p2.x*_cos - p2.y*_sin),(p2.x*_sin + p2.y*_cos));
		dk = abs(r2.x - r1.x) + abs(r2.y - r1.y);
	}

	ell._rel = min(1.f, ((di + dj + dk) / (3*(ell._A + ell._B))));
	
	//not enough reliable
	if(ell._rel < _fMinReliability) return;

	//the tentative detection has been confirmed. Save it!
	ellipses.push_back(ell);
};

// Get the coordinates of the center, given the intersection of the estimated lines. See Sect [] in the paper.
Point2f CEllipseDetectorLX::GetCenterCoordinates	(	EllipseData& data_ij,
														EllipseData& data_ik
													)
{
	float xx[7];
	float yy[7];

	xx[0] = data_ij.center.x;
	xx[1] = data_ik.center.x;
	yy[0] = data_ij.center.y;
	yy[1] = data_ik.center.y;
	
	{
		//1-1
		float q2 = data_ij.t1;
		float q4 = data_ik.t1;
		Point2f& M12 = data_ij.M1;
		Point2f& M34 = data_ik.M1;

		float invDen = 1 / (q2 - q4);
		xx[2] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[2] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	{
		//1-2
		float q2 = data_ij.t1;
		float q4 = data_ik.t2;
		Point2f& M12 = data_ij.M1;
		Point2f& M34 = data_ik.M2;

		float invDen = 1 / (q2 - q4);
		xx[3] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[3] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	{
		//2-2
		float q2 = data_ij.t2;
		float q4 = data_ik.t2;
		Point2f& M12 = data_ij.M2;
		Point2f& M34 = data_ik.M2;

		float invDen = 1 / (q2 - q4);
		xx[4] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[4] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	{
		//2-1
		float q2 = data_ij.t2;
		float q4 = data_ik.t1;
		Point2f& M12 = data_ij.M2;
		Point2f& M34 = data_ik.M1;

		float invDen = 1 / (q2 - q4);
		xx[5] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[5] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	xx[6] = (xx[0] + xx[1]) * 0.5f;
	yy[6] = (yy[0] + yy[1]) * 0.5f;


	//MEDIAN
	nth_element(xx, xx+3,xx+7);
	nth_element(yy, yy+3,yy+7);
	float xc = xx[3];
	float yc = yy[3];

	return Point2f(xc,yc);
};


//verify triplets of arcs with convexity: i=1, j=2, k=4
void CEllipseDetectorLX::Triplets124	(	VVP& pi,
											VVP& pj,
											VVP& pk,
											unordered_map<uint, EllipseData>& data,
											vector<CEllipse>& ellipses
										)
{
	// get arcs length
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());

	//for each edge i
	for(ushort i=0; i<sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pil = edge_i[sz_ei - 1];

		//1,2 -> reverse 1, swap
		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());

		//for each edge j
		for(ushort j=0; j<sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjl = edge_j[sz_ej - 1];

			//CONSTRAINTS on position
			if(pjl.x > pif.x) //is right				
			{				
				//discard
				continue;
			}
			
			uint key_ij = GenerateKey(PAIR_12, i, j);

			//for each edge k
			for(ushort k=0; k<sz_k; ++k)
			{
				VP& edge_k = pk[k];
				ushort sz_ek = ushort(edge_k.size());

				Point& pkf = edge_k[0];
				Point& pkl = edge_k[sz_ek - 1];
								
				//CONSTRAINTS on position
				if	(	(pkl.y < pil.y) ||		//is above i
						(pkl.y < pjl.y)	||		//is above j
						(pkl.x < pjf.x)			//is left j
					)
				{					
					//discard
					continue;
				}

				uint key_ik = GenerateKey(PAIR_14, i, k);

				//find centers
			
				EllipseData data_ij, data_ik;							

				//if the data for the pair i-j have not been computed yet
				if(data.count(key_ij) == 0)
				{					
					//1,2 -> reverse 1, swap

					//compute data!
					GetFastCenter(edge_j, rev_i, data_ij);
					//insert computed data in the hash table
					data.insert(pair<uint,EllipseData>(key_ij,data_ij));
				}
				else
				{
					//otherwise, just lookup the data in the hash table
					data_ij = data.at(key_ij);
				}

				//if the data for the pair i-k have not been computed yet
				if(data.count(key_ik) == 0)
				{
					//1,4 -> ok

					//compute data!
					GetFastCenter(edge_i, edge_k, data_ik);	
					//insert computed data in the hash table
					data.insert(pair<uint,EllipseData>(key_ik,data_ik));
				}
				else
				{
					//otherwise, just lookup the data in the hash table
					data_ik = data.at(key_ik);
				}

				//INVALID CENTERS
				if(!data_ij.isValid || !data_ik.isValid)
				{
					continue;
				}

				//Selection strategy - Step 3. See Sect [] in the paper
				//the computer centers are not close enough
				if(ed2(data_ij.center, data_ik.center) > _fMaxCenterDistance2)
				{
					//discard
					continue;
				}

				// if all constraints of the selection strategy have been satisfied, 
				// we can start estimating the ellipse parameters

				//FIND ELLIPSE PARAMETERS
				
				// get the coordinates of the center (xc, yc)
				Point2f center = GetCenterCoordinates(data_ij, data_ik);
				//find remaining paramters (A,B,rho)
				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);
			}				
		}
	}
};



void CEllipseDetectorLX::Triplets231	(	VVP& pi,
											VVP& pj,
											VVP& pk,
											unordered_map<uint, EllipseData>& data,
											vector<CEllipse>& ellipses
										)
{
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());

	//for each edge i
	for(ushort i=0; i<sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pil = edge_i[sz_ei - 1];

		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());

		//for each edge j
		for(ushort j=0; j<sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjl = edge_j[sz_ej - 1];			

			//CONSTRAINTS on position
			if(pjf.y < pif.y) //is above
				
			{
				//discard
				continue;
			}

			VP rev_j(edge_j.size());
			reverse_copy(edge_j.begin(), edge_j.end(), rev_j.begin());

			uint key_ij = GenerateKey(PAIR_23, i, j);

			//for each edge k
			for(ushort k=0; k<sz_k; ++k)
			{
				VP& edge_k = pk[k];
				ushort sz_ek = ushort(edge_k.size());

				Point& pkf = edge_k[0];
				Point& pkl = edge_k[sz_ek - 1];

				//CONSTRAINTS on position
				if	(	(pkf.x < pil.x) ||		//is left i
						(pkl.x < pjf.x)	||		//is left j
						(pkf.y > pjl.y)			//is below j
					)
				{
					//discard
					continue;
				}

				uint key_ik = GenerateKey(PAIR_12, k, i);
				
				//find centers

				EllipseData data_ij, data_ik;

				if(data.count(key_ij) == 0)
				{
					//2,3 -> reverse 2,3
					
					GetFastCenter(rev_i, rev_j, data_ij);
					data.insert(pair<uint,EllipseData>(key_ij,data_ij));
				}
				else
				{
					data_ij = data.at(key_ij);
				}

				if(data.count(key_ik) == 0)
				{
					//2,1 -> reverse 1
					VP rev_k(edge_k.size());
					reverse_copy(edge_k.begin(), edge_k.end(), rev_k.begin());

					GetFastCenter(edge_i, rev_k, data_ik);
					data.insert(pair<uint,EllipseData>(key_ik,data_ik));
				}
				else
				{
					data_ik = data.at(key_ik);
				}

				//INVALID CENTERS
				if(!data_ij.isValid || !data_ik.isValid)
				{
					continue;
				}

				//CONSTRAINT ON CENTERS
				if(ed2(data_ij.center, data_ik.center) > _fMaxCenterDistance2)
				{
					//discard
					continue;
				}

				//FIND ELLIPSE PARAMETERS
				Point2f center = GetCenterCoordinates(data_ij, data_ik);

				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);

			}
		}
	}
};


void CEllipseDetectorLX::Triplets342	(	VVP& pi,
											VVP& pj,
											VVP& pk,
											unordered_map<uint, EllipseData>& data,
											vector<CEllipse>& ellipses
										)
{
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());
	
	//for each edge i
	for(ushort i=0; i<sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pil = edge_i[sz_ei - 1];

		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());
		
		//for each edge j
		for(ushort j=0; j<sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjl = edge_j[sz_ej - 1];


			//CONSTRAINTS on position
			if	(pjf.x < pil.x) 		//is left
			{
				//discard
				continue;
			}
			VP rev_j(edge_j.size());
			reverse_copy(edge_j.begin(), edge_j.end(), rev_j.begin());

			uint key_ij = GenerateKey(PAIR_34, i, j);

			//for each edge k
			for(ushort k=0; k<sz_k; ++k)
			{
				VP& edge_k = pk[k];
				ushort sz_ek = ushort(edge_k.size());

				Point& pkf = edge_k[0];
				Point& pkl = edge_k[sz_ek - 1];

				//CONSTRAINTS on position
				if	(	(pkf.y > pif.y) ||		//is below i
						(pkf.y > pjf.y)	||		//is below j
						(pkf.x > pjl.x)			//is right j
					)
				{
					//discard
					continue;
				}				

				uint key_ik = GenerateKey(PAIR_23, k, i);

				//find centers

				EllipseData data_ij, data_ik;

				if(data.count(key_ij) == 0)
				{
					//3,4 -> reverse 4
					

					GetFastCenter(edge_i, rev_j, data_ij);

					data.insert(pair<uint,EllipseData>(key_ij,data_ij));
				}
				else
				{
					data_ij = data.at(key_ij);
				}

				if(data.count(key_ik) == 0)
				{
					//3,2 -> reverse 3,2
					
					VP rev_k(edge_k.size());
					reverse_copy(edge_k.begin(), edge_k.end(), rev_k.begin());

					GetFastCenter(rev_i, rev_k, data_ik);

					data.insert(pair<uint,EllipseData>(key_ik,data_ik));
				}
				else
				{
					data_ik = data.at(key_ik);
				}

				
				//INVALID CENTERS
				if(!data_ij.isValid || !data_ik.isValid)
				{
					continue;
				}

				//CONSTRAINT ON CENTERS
				if(ed2(data_ij.center, data_ik.center) > _fMaxCenterDistance2)
				{
					//discard
					continue;
				}

				//FIND ELLIPSE PARAMETERS
				Point2f center = GetCenterCoordinates(data_ij, data_ik);				
				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);
			}			
		}
				
	}
};


void CEllipseDetectorLX::Triplets413	(	VVP& pi,
											VVP& pj,
											VVP& pk,
											unordered_map<uint, EllipseData>& data,
											vector<CEllipse>& ellipses
										)
{
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());

	//for each edge i
	for(ushort i=0; i<sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pil = edge_i[sz_ei - 1];

		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());

		//for each edge j
		for(ushort j=0; j<sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjl = edge_j[sz_ej - 1];


			//CONSTRAINTS on position
			if(pjl.y > pil.y)  		//is below
			{
				//discard
				continue;
			}

			uint key_ij = GenerateKey(PAIR_14, j, i);

			//for each edge k
			for(ushort k=0; k<sz_k; ++k)
			{
				VP& edge_k = pk[k];
				ushort sz_ek = ushort(edge_k.size());

				Point& pkf = edge_k[0];
				Point& pkl = edge_k[sz_ek - 1];


				//CONSTRAINTS on position
				if	(	(pkl.x > pif.x) ||		//is right i
						(pkl.x > pjl.x)	||		//is right j
						(pkl.y < pjf.y)		//is above j
					)
				{
					//discard
					continue;
				}

				uint key_ik = GenerateKey(PAIR_34, k, i);

				//find centers

				EllipseData data_ij, data_ik;

				if(data.count(key_ij) == 0)
				{
					//4,1 -> OK
					GetFastCenter(edge_i, edge_j, data_ij);
					data.insert(pair<uint,EllipseData>(key_ij,data_ij));
				}
				else
				{
					data_ij = data.at(key_ij);
				}

				if(data.count(key_ik) == 0)
				{
					//4,3 -> reverse 4
					GetFastCenter(rev_i, edge_k, data_ik);
					data.insert(pair<uint,EllipseData>(key_ik,data_ik));
				}
				else
				{
					data_ik = data.at(key_ik);
				}

				//INVALID CENTERS
				if(!data_ij.isValid || !data_ik.isValid)
				{					
					continue;
				}

				//CONSTRAIN ON CENTERS
				if(ed2(data_ij.center, data_ik.center) > _fMaxCenterDistance2)
				{
					//discard
					continue;
				}

				//FIND ELLIPSE PARAMETERS
				Point2f center = GetCenterCoordinates(data_ij, data_ik);

				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);

			}
		}
	}
};


void CEllipseDetectorLX::RemoveShortEdges(Mat1b& edges, Mat1b& clean)
{
	VVP contours;

	//labeling and contraints on length
	Labeling(edges, contours, _iMinEdgeLength);

	int iContoursSize = contours.size();
	for(int i=0; i<iContoursSize; ++i)
	{
		VP& edge = contours[i];
		unsigned szEdge = edge.size();

		//constraint on axes aspect ratio
		RotatedRect oriented = minAreaRect(edge);
		if(	oriented.size.width < _fMinOrientedRectSide ||
			oriented.size.height < _fMinOrientedRectSide ||
			oriented.size.width > oriented.size.height * _fMaxRectAxesRatio ||
			oriented.size.height > oriented.size.width * _fMaxRectAxesRatio )
		{
			continue;
		}

		for(unsigned j=0; j<szEdge; ++j)
		{
			clean(edge[j]) = (uchar)255;
		}
	}
}



void CEllipseDetectorLX::PrePeocessing	(	Mat1b& I,
												Mat1b& DP,
												Mat1b& DN
											)
{
	//smooth image
	GaussianBlur(I, I, _szPreProcessingGaussKernelSize, _dPreProcessingGaussSigma);

	//temp variables
	Mat1b E;				//edge mask
	Mat1s DX, DY;			//sobel derivatives

	//detect edges
	//Canny2(I, E, DX, DY, _dPreProcessingCannyMinThreshold, _dPreProcessingCannyMaxThreshold, 3, false);
	
	//use automatic thresholds. Slower but more general
	Canny3(I, E, DX, DY, 3, false);

	//for each edge points, compute the edge direction
	for(int i=0; i<_szImg.height; ++i)
	{		
		short* _dx		= DX.ptr<short>(i);
		short* _dy		= DY.ptr<short>(i);
		uchar* _e		= E.ptr<uchar>(i);
		uchar* _dp		= DP.ptr<uchar>(i);
		uchar* _dn		= DN.ptr<uchar>(i);

		for(int j=0; j<_szImg.width; ++j)
		{
			if(!((_e[j] <= 0) || (_dx[j] == 0) || (_dy[j] == 0)))
			{
				// angle of the tangent
				float phi = -(float(_dx[j]) / float(_dy[j]));
				
				// along positive or negative diagonal
				if		(phi > 0)	_dp[j] = (uchar)255;
				else if	(phi < 0)	_dn[j] = (uchar)255;
			}
		}
	}
};


void CEllipseDetectorLX::DetectAfterPreProcessing(vector<CEllipse>& ellipses, Mat1b& E, Mat1f& PHI)
{
	// set the image size
	_szImg = E.size();

	// initialize temporary data structures
	Mat1b DP	= Mat1b::zeros(_szImg);		// arcs along positive diagonal
	Mat1b DN	= Mat1b::zeros(_szImg);		// arcs along negative diagonal
	
	for(int i=0; i<_szImg.height; ++i)
	{
		float* _phi		= PHI.ptr<float>(i);
		uchar* _e		= E.ptr<uchar>(i);
		uchar* _dp		= DP.ptr<uchar>(i);
		uchar* _dn		= DN.ptr<uchar>(i);

		for(int j=0; j<_szImg.width; ++j)
		{
			if((_e[j] > 0) && (_phi[j] != 0))
			{
				// angle
				float phi = std::atan(-1.f / _phi[j]);

				// along positive or negative diagonal
				if		(phi > 0)	_dp[j] = (uchar)255;
				else if	(phi < 0)	_dn[j] = (uchar)255;
			}
		}
	}	

	//initialize accumulator dimensions
	ACC_N_SIZE = 101;
	ACC_K_SIZE = 180;
	ACC_A_SIZE = max(_szImg.height, _szImg.width);

	//allocate accumulators
	accN = new int[ACC_N_SIZE];
	accK = new int[ACC_K_SIZE];
	accA = new int[ACC_A_SIZE];

	//other temporary 
	VVP points_1, points_2, points_3, points_4;		//vector of points, one for each convexity class
	unordered_map<uint, EllipseData> centers;		//hash map for reusing already computed EllipseData

	//start timer
	Tic();

	//detect edges and find convexities
	DetectEdges13(DP, points_1, points_3);
	DetectEdges24(DN, points_2, points_4);
		
	//find triplets
	Triplets124(points_1, points_2, points_4, centers, ellipses);
	Triplets231(points_2, points_3, points_1, centers, ellipses);
	Triplets342(points_3, points_4, points_2, centers, ellipses);
	Triplets413(points_4, points_1, points_3, centers, ellipses);

	//stop timer
	Toc();	

	//sort detected ellipses with respect to score
	sort(ellipses.begin(), ellipses.end());

	//free accumulator memory
	delete [] accN;
	delete [] accK;
	delete [] accA;

	//cluster detections
	ClusterEllipses(ellipses);
};


void CEllipseDetectorLX::Detect	(	Mat1b& I,
										vector<CEllipse>& ellipses
									)
{
	// set the image size
	_szImg = I.size();

	// initialize temporary data structures
	Mat1b DP	= Mat1b::zeros(_szImg);		// arcs along positive diagonal
	Mat1b DN	= Mat1b::zeros(_szImg);		// arcs along negative diagonal

	//initialize accumulator dimensions
	ACC_N_SIZE = 101;
	ACC_K_SIZE = 180;
	ACC_A_SIZE = max(_szImg.height, _szImg.width);

	//allocate accumulators
	accN = new int[ACC_N_SIZE];
	accK = new int[ACC_K_SIZE];
	accA = new int[ACC_A_SIZE];
	
	//other temporary 
	VVP points_1, points_2, points_3, points_4;		//vector of points, one for each convexity class
	unordered_map<uint, EllipseData> centers;		//hash map for reusing already computed EllipseData

	//Preprocessing
	//From input image I, find edge point with coarse convexity along positive (DP) or negative (DN) diagonal
	PrePeocessing(I, DP, DN);
	
	//start timer
	Tic();

	//detect edges and find convexities
	DetectEdges13(DP, points_1, points_3);
	DetectEdges24(DN, points_2, points_4);
			
	//find triplets
	Triplets124(points_1, points_2, points_4, centers, ellipses);
	Triplets231(points_2, points_3, points_1, centers, ellipses);
	Triplets342(points_3, points_4, points_2, centers, ellipses);
	Triplets413(points_4, points_1, points_3, centers, ellipses);

	//stop timer
	Toc();

	//sort detected ellipses with respect to score
	sort(ellipses.begin(), ellipses.end());

	//free accumulator memory
	delete [] accN;
	delete [] accK;
	delete [] accA;

	//cluster detections
	ClusterEllipses(ellipses);
};

//Ellipse clustering procedure. See Sect [] in the paper.
void CEllipseDetectorLX::ClusterEllipses(vector<CEllipse>& ellipses)
{	
	float _fMaxCenterDistance	= 16.f;
	float _fMaxEccDistance		= 0.2f;
	float _fMaxADistance		= 0.1f;
	float _fMaxAngleDistance	= 0.1f;
	float _fThresholdCircle		= 0.8f;

	int iNumOfEllipses = int(ellipses.size());
	if(iNumOfEllipses == 0) return;

	//the first ellipse is assigned to a cluster
	vector<CEllipse> clusters;
	clusters.push_back(ellipses[0]);

	bool bFoundCluster = false;

	for(int i=1; i<iNumOfEllipses; ++i)
	{
		CEllipse& e1 = ellipses[i];

		int sz_clusters = clusters.size();

		float Decc1 = e1._B / e1._A;

		bool bFoundCluster = false;
		for(int j=0; j<sz_clusters; ++j)
		{
			CEllipse& e2 = clusters[j];

			//centers
			float Dx = abs(e1._xc - e2._xc);
			float Dy = abs(e1._yc - e2._yc);

			if(Dx > _fMaxCenterDistance || Dy > _fMaxCenterDistance)
			{
				//not same cluster
				continue;
			}

			//axes
			float Da = 1.f - (min(e1._A, e2._A) / max(e1._A, e2._A));
			if(Da > _fMaxADistance)
			{
				//not same cluster
				continue;
			}

			//eccentricity
			float Decc2 = e2._B / e2._A;

			float De = 1.f - (min(Decc1,Decc2) / max(Decc1,Decc2));
			if(De > _fMaxEccDistance) 
			{
				//not same cluster
				continue;
			}			

			//angle
			float Drho = fmod(abs(e1._rho - e2._rho), float(CV_PI)) / float(CV_PI);
			if(Drho > _fMaxAngleDistance)
			{
				if(Decc1 < _fThresholdCircle && Decc2 < _fThresholdCircle)
				{
					//not same cluster
					continue;
				}
			}
						

			//same cluster as e2
			bFoundCluster = true;
			//discard, no need to create a new cluster
			break;
		}

		if(!bFoundCluster)
		{
			//create a new cluster			
			clusters.push_back(e1);
		}
	}

	clusters.swap(ellipses);
};

//Draw at most iTopN detected ellipses.
void CEllipseDetectorLX::DrawDetectedEllipses(Mat3b& output, vector<CEllipse>& ellipses, int iTopN, int thickness)
{	
	int sz_ell = int(ellipses.size());
	int n = (iTopN==0) ? sz_ell : min(iTopN, sz_ell);
    int radius_circle=1;
    int x_center=320;
    int y_center=240;
    int xaxis=0;
    int yaxis=0;

    //const char *label="I love it";
    CvFont font;
    IplImage* outputt = new IplImage(output);


   // cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0.0, 1, 8);
       // sprintf(result, "color: %s", label);


	for(int i=0; i < n; ++i)
	{
        CEllipse& e = ellipses[sz_ell - i - 1];

        int g = cvRound(e._score * 255.f);
        Scalar color(0, g, 0);
        ellipse(output, Point(e._xc, e._yc), Size(e._A, e._B), e._rho*180.f/CV_PI, 0.0, 360.0, color, thickness);
                circle( output,Point(x_center,y_center), radius_circle, color, thickness );
        circle( output, Point(e._xc, e._yc), radius_circle, color, thickness );
xaxis=xaxis+e._xc;
yaxis=yaxis+e._yc;
        //locationdisp.x=;
        //locationdisp.y=e._yc;

        //sprintf(result, "color: %f", e._xc);
        //cvPutText( output, result, cvPoint(e._xc, e._yc), &font, color );
	}
    if(n>1){
    locationn.xaxis=xaxis/n;
    locationn.yaxis=yaxis/n;}
    else{
    locationn.xaxis=0;
    locationn.yaxis=0;
}

}

Mat3b CEllipseDetectorLX::Onvideotracking(Mat3b image)
{
        CEllipseDetectorLX yaed;

        Mat1b gray;

        cvtColor(image, gray, CV_BGR2GRAY);

        vector<CEllipse> ellipses;

        //Find Ellipses
        yaed.Detect(gray, ellipses);
        yaed.DrawDetectedEllipses(image,ellipses);

        return image;
        //cv::imshow("Output", image);
}


VideoCapture CEllipseDetectorLX::opencamera(void)
{
VideoCapture cap(1);

    return cap;
}


location CEllipseDetectorLX::locations(void)
{
    return locationn;
}
