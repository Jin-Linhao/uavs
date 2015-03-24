
#ifndef RECTANGLEDETECTOR_H_
#define RECTANGLEDETECTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
using namespace std;

class CRectangleDetector {
public:
	CRectangleDetector();
	virtual ~CRectangleDetector();

	static void help();

	void FindSquares( const cv::Mat& image,
			vector<vector<cv::Point> >& squares,
			vector<vector<cv::Point> > colorSquares[] );
    cv::Mat DrawSquares( cv::Mat& image, const vector<vector<cv::Point> > squares[] );
    cv::Mat DrawSquares( cv::Mat& image, const vector<vector<cv::Point> >& squares );


public: // Fields, structs

	struct Center
	{
	    cv::Point2f location;
	    float radius;
	    // double confidence;
	};

private: // Methods

	double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );
	void FilterByMaxSize(const vector<vector<cv::Point> > &colorSquares,
			vector<vector<cv::Point> > &filteredBySizeSquares,
			const int maxsize = 100000);
	void FilterByMaxSize(const vector<vector<cv::Point> > colorSquares[],
			vector<vector<cv::Point> > filteredBySizeSquares[],
			const int maxsize = 100000);
	void SortByCenters(const vector<vector<cv::Point> > &colorSquares,
			vector<vector<vector<cv::Point> > > &sortedSquares,
			const int minDist = 50);
	void SortByCenters(const vector<vector<cv::Point> > colorSquares[],
			vector<vector<vector<cv::Point> > > sortededByCentersSquares[],
			const int minDist = 50);
	void DumpSortedCenterData(const vector<vector<vector<cv::Point> > > &sortedByCenterSquares);
	void DumpSortedCenterData(const vector<vector<vector<cv::Point> > > sortedByCenterSquares[]);
	void ConsolidateSquares(const vector<vector<vector<cv::Point> > > sortedSquares[],
			vector<vector<cv::Point> > &consolidatedSquares);
	void FilterByBGR(const cv::Mat& image, const vector<vector<cv::Point> > &sortedSquares,
			vector<vector<cv::Point> > &sortedByRGBSquares,
			const vector<cv::Scalar> &colorRange);
	void FilterSquares(const cv::Mat& image,
			vector<vector<cv::Point> > colorSquares[],
			vector<vector<cv::Point> > &squares);



private: // Fields

	int m_thresh;
	int m_numThreshIters;
	std::string m_wndName;
	int m_numOfColorPlanes;

};

#endif /* RECTANGLEDETECTOR_H_ */
