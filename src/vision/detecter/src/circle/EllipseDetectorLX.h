#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
//#include "opencv2/core/internal.hpp"
#include <stdio.h>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "Ellipse.h"
#include "common.h"
#include <time.h>




#include <stdio.h>
#include <math.h>
#include <deque>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"


using namespace std;
using namespace cv;

struct locationdis{
    const char *x;
    const char *y;
};

struct location{
    int xaxis;
    int yaxis;

};

struct EllipseData
{
	float t1;
	float m1;
	float t2;
	float m2;
	Point2f M1;
	Point2f M2;
	Point2f center;
	bool isValid;
	vector<float> slopes1;
	vector<float> slopes2;
};


class CEllipseDetectorLX
{
	// Parameters

	// Preprocessing - Gaussian filter. See Sect [] in the paper
	Size	_szPreProcessingGaussKernelSize;	// size of the Gaussian filter in preprocessing step
	double	_dPreProcessingGaussSigma;			// sigma of the Gaussian filter in the preprocessing step
		
	

	// Selection strategy - Step 1 - Discard noisy or straight arcs. See Sect [] in the paper
	int		_iMinEdgeLength;					// minimum edge size				
	float	_fMinOrientedRectSide;				// minumum size of the oriented bounding box containing the arc
	float	_fMaxRectAxesRatio;					// maximum aspect ratio of the oriented bounding box containing the arc

	// Selection strategy - Step 2 - Remove according to mutual convexities. See Sect [] in the paper
	float _fThPosition;

	// Selection Strategy - Step 3 - Number of points considered for slope estimation when estimating the center. See Sect [] in the paper
	int _iNs;									// Find at most Ns parallel chords.

	// Selection strategy - Step 3 - Discard pairs of arcs if their estimated center is not close enough. See Sect [] in the paper
	float	_fMaxCenterDistance;				// maximum distance in pixel between 2 center points
	float	_fMaxCenterDistance2;				// _fMaxCenterDistance * _fMaxCenterDistance

	// Validation - Points within a this threshold are considered to lie on the ellipse contour. See Sect [] in the paper
	float	_fDistanceToEllipseContour;			// maximum distance between a point and the contour. See equation [] in the paper

	// Validation - Assign a score. See Sect [] in the paper
	float	_fMinScore;							// minimum score to confirm a detection
	float	_fMinReliability;					// minimum auxiliary score to confirm a detection


	// auxiliary variables
	Size	_szImg;			// input image size
	double _dExecTime;		// execution time

	int ACC_N_SIZE;			// size of accumulator N = B/A
	int ACC_K_SIZE;			// size of accumulator K = tan(rho)
	int ACC_A_SIZE;			// size of accumulator A

	int* accN;				// pointer to accumulator N
	int* accK;				// pointer to accumulator K
	int* accA;				// pointer to accumulator A



public:

	//Constructor and Destructor
    CEllipseDetectorLX(void);

    ~CEllipseDetectorLX(void);

    Mat3b Onvideotracking(Mat3b);

    VideoCapture opencamera(void);

    location locations(void);

    void DetectAfterPreProcessing(vector<CEllipse>& ellipses, Mat1b& E, Mat1f& PHI);

	//Detect the ellipses in the gray image
	void Detect(Mat1b& gray, vector<CEllipse>& ellipses);
	
	//Draw the first iTopN ellipses on output
	void DrawDetectedEllipses(Mat3b& output, vector<CEllipse>& ellipses, int iTopN=0, int thickness=2);
	
	//Set the parameters of the detector
	void SetParameters	(	Size	szPreProcessingGaussKernelSize,
							double	dPreProcessingGaussSigma,
							double 	fThPosition,
							float	fMaxCenterDistance,
							int		iMinEdgeLength,
							float	fMinOrientedRectSide,
							float	fDistanceToEllipseContour,
							float	fMinScore,
							float	fMinReliability,
							int     iNs
						);

	//return the execution time
	double GetExecTime()
	{
		return _dExecTime; 
	}
	
private:

	//keys for hash table
	static const ushort PAIR_12 = 0x00;
	static const ushort PAIR_23 = 0x01;
	static const ushort PAIR_34 = 0x02;
	static const ushort PAIR_14 = 0x03;

	//generate keys from pair and indicse
	uint inline GenerateKey(uchar pair, ushort u, ushort v);




	void PrePeocessing(Mat1b& I, Mat1b& DP, Mat1b& DN);

	void RemoveShortEdges(Mat1b& edges, Mat1b& clean);

	void ClusterEllipses(vector<CEllipse>& ellipses);

	int FindMaxK(const vector<int>& v) const;
	int FindMaxN(const vector<int>& v) const;
	int FindMaxA(const vector<int>& v) const;

	int FindMaxK(const int* v) const;
	int FindMaxN(const int* v) const;
	int FindMaxA(const int* v) const;

	float GetMedianSlope(vector<Point2f>& med, Point2f& M, vector<float>& slopes);
	void GetFastCenter	(vector<Point>& e1, vector<Point>& e2, EllipseData& data);
	

	void DetectEdges13(Mat1b& DP, VVP& points_1, VVP& points_3);
	void DetectEdges24(Mat1b& DN, VVP& points_2, VVP& points_4);

	void FindEllipses	(	Point2f& center,
							VP& edge_i,
							VP& edge_j,
							VP& edge_k,
							EllipseData& data_ij,
							EllipseData& data_ik,
							vector<CEllipse>& ellipses
						);

	Point2f GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik);
	Point2f _GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik);

	

	void Triplets124	(	VVP& pi,
							VVP& pj,
							VVP& pk,
							unordered_map<uint, EllipseData>& data,
							vector<CEllipse>& ellipses
						);

	void Triplets231	(	VVP& pi,
							VVP& pj,
							VVP& pk,
							unordered_map<uint, EllipseData>& data,
							vector<CEllipse>& ellipses
						);

	void Triplets342	(	VVP& pi,
							VVP& pj,
							VVP& pk,
							unordered_map<uint, EllipseData>& data,
							vector<CEllipse>& ellipses
						);

	void Triplets413	(	VVP& pi,
							VVP& pj,
							VVP& pk,
							unordered_map<uint, EllipseData>& data,
							vector<CEllipse>& ellipses
						);

	void Tic()
	{
		_dExecTime = (double)cv::getTickCount();
	}

	void Toc()
	{
		_dExecTime = ((double)cv::getTickCount() - _dExecTime)*1000./cv::getTickFrequency();
	}




};

