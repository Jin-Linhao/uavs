
#include "RectangleDetector.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
using namespace std;

CRectangleDetector::CRectangleDetector()
	: m_thresh(50),
	  m_numThreshIters(11),
	  m_wndName("Rectangle Detector"),
	  m_numOfColorPlanes(3)
{
	// Do Nothing
}

CRectangleDetector::~CRectangleDetector()
{
	// Do Nothing
}

// static
void CRectangleDetector::help()
{
    std::cout <<
    "\nA program using pyramid scaling, Canny, contours, contour simpification and\n"
    "memory storage (it's got it all folks) to find\n"
    "squares in a list of images pic1-6.png\n"
    "Returns sequence of squares detected on the image.\n"
    "the sequence is stored in the specified memory storage\n"
    "Call:\n"
    "./squares\n"
    "Using OpenCV version %s\n" << CV_VERSION << "\n" << std::endl;
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double CRectangleDetector::angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}



void CRectangleDetector::FilterByMaxSize(const vector<vector<cv::Point> > &colorSquares,
		vector<vector<cv::Point> > &filteredBySizeSquares,
		const int maxsize /*100000*/)
{
	// Eliminate squares that are greater than maximum allowable size (min size has already been filtered out)
	vector<vector<cv::Point> >::const_iterator citer = colorSquares.begin();
	for (; citer != colorSquares.end(); ++citer)
	{
		if (fabs(cv::contourArea(cv::Mat(*citer))) < maxsize)
		{
			filteredBySizeSquares.push_back(*citer);
		}
	}
}



void CRectangleDetector::FilterByMaxSize(const vector<vector<cv::Point> > colorSquares[],
		vector<vector<cv::Point> > filteredBySizeSquares[],
		int maxsize /*100000*/)
{
	// Eliminate squares that are greater than maximum allowable size (min size has already been filtered out)
	for (int c=0; c<m_numOfColorPlanes; ++c)
	{
		FilterByMaxSize(colorSquares[c], filteredBySizeSquares[c], maxsize);
	}
}



void CRectangleDetector::SortByCenters(const vector<vector<cv::Point> > &colorSquares,
		vector<vector<vector<cv::Point> > > &sortedSquares,
		const int minDist /*50*/)
{
	for (size_t i=0; i<colorSquares.size(); ++i)
	{
		bool isNew = true;
		Center curCenter;
		cv::minEnclosingCircle(cv::Mat(colorSquares[i]), curCenter.location, curCenter.radius);

		for (size_t j=0; j<sortedSquares.size() && isNew; ++j)
		{
			Center sortedCenter;

			// For this algorithm we pick the sorted square in the middle of the array to check against since they are sorted by radius size
			cv::minEnclosingCircle(cv::Mat(sortedSquares[j][ sortedSquares[j].size()/2 ]), sortedCenter.location, sortedCenter.radius);

			double dist = norm(sortedCenter.location - curCenter.location);
			isNew = dist >= minDist && dist >= sortedCenter.radius && dist >= curCenter.radius;
			if (!isNew)
			{
				// Determine where this radius fits in the group
				size_t k = sortedSquares[j].size() - 1;
				cv::minEnclosingCircle(cv::Mat(sortedSquares[j][k]), sortedCenter.location, sortedCenter.radius);
				while( k > 0 && curCenter.radius<sortedCenter.radius )
				{
					k--;
					cv::minEnclosingCircle(cv::Mat(sortedSquares[j][k]), sortedCenter.location, sortedCenter.radius);
				}
				if (curCenter.radius>sortedCenter.radius)
				{
					++k;
				}
				sortedSquares[j].insert(sortedSquares[j].begin() + k, colorSquares[i]);
			}
		}
		if (isNew)
		{
			// Start a new group of squares
			sortedSquares.push_back(vector<vector<cv::Point> > (1, colorSquares[i]));
		}
	}
}


void CRectangleDetector::SortByCenters(const vector<vector<cv::Point> > colorSquares[],
		vector<vector<vector<cv::Point> > > sortededByCentersSquares[],
		const int minDist /*50*/)
{
	// Eliminate squares that are greater than maximum allowable size (min size has already been filtered out)
	for (int c=0; c<m_numOfColorPlanes; ++c)
	{
		SortByCenters(colorSquares[c], sortededByCentersSquares[c], minDist);
	}
}



void CRectangleDetector::DumpSortedCenterData(const vector<vector<vector<cv::Point> > > &sortedByCenterSquares)
{
	vector<vector<vector<cv::Point> > >::const_iterator cCenterDataIter = sortedByCenterSquares.begin();
	for (; cCenterDataIter != sortedByCenterSquares.end(); ++cCenterDataIter)
	{
		vector<vector<cv::Point> >::const_iterator cCenterIter = cCenterDataIter->begin();
		for (; cCenterIter != cCenterDataIter->end(); ++cCenterIter)
		{
			Center center;
			cv::minEnclosingCircle(cv::Mat(*cCenterIter), center.location, center.radius);
			cout << "L: " << center.location << " R: " << center.radius << "\t";
		}
		cout << endl;
	}

	cout << endl;
}



void CRectangleDetector::DumpSortedCenterData(const vector<vector<vector<cv::Point> > > sortedByCenterSquares[])
{
	// Eliminate squares that are greater than maximum allowable size (min size has already been filtered out)
	for (int c=0; c<m_numOfColorPlanes; ++c)
	{
		cout << "Dumping Data from layer \"" << c << "\" sortedByCenterSquares:" << endl;
		DumpSortedCenterData(sortedByCenterSquares[c]);
	}
}



void CRectangleDetector::ConsolidateSquares(const vector<vector<vector<cv::Point> > > sortedSquares[],
		vector<vector<cv::Point> > &consolidatedSquares)
{
	// Make sure we don't have any residual data in this vector or we could end up with repeats
	consolidatedSquares.clear();

	vector<vector<vector<cv::Point> > > sortedByCenterSquares;
	const int minDist = 50;

	// For this algorithm we simply select the square located in the middle of the vector since they are sorted by radius size

	// Pull center from each plane into one vector
	for (int c=0; c<m_numOfColorPlanes; ++c)
	{
		vector<vector<vector<cv::Point> > >::const_iterator cSortedIter = sortedSquares[c].begin();
		for (; cSortedIter != sortedSquares[c].end(); ++cSortedIter)
		{
			size_t middle = cSortedIter->size()/2;
			vector<cv::Point> middleSquare = cSortedIter->at(middle);
			consolidatedSquares.push_back(middleSquare);
		}
	}

	// Reduce to one unique square
	//	Color information is gone by this point
	SortByCenters(consolidatedSquares, sortedByCenterSquares, minDist);

	// DumpSortedCenterData(sortedByCenterSquares);

	consolidatedSquares.clear();

	vector<vector<vector<cv::Point> > >::const_iterator cSortedIter = sortedByCenterSquares.begin();
	for (; cSortedIter != sortedByCenterSquares.end(); ++cSortedIter)
	{
		size_t middle = cSortedIter->size()/2;
		vector<cv::Point> middleSquare = cSortedIter->at(middle);
		consolidatedSquares.push_back(middleSquare);
	}
}


void CRectangleDetector::FilterByBGR(const cv::Mat& image, const vector<vector<cv::Point> > &sortedSquares,
		vector<vector<cv::Point> > &sortedByRGBSquares,
		const vector<cv::Scalar> &colorRange)
{
	// Make sure that we don't have any residual data in the returned vector
	sortedByRGBSquares.clear();

	cv::Mat upperMat;
	cv::Mat lowerMat;
	cv::Mat resultMat;

	bool isInRange(true);

	vector<vector<cv::Point> >::const_iterator cSortedSquaresIter = sortedSquares.begin();
	for (; cSortedSquaresIter != sortedSquares.end(); ++cSortedSquaresIter)
	{
		cv::Rect rect = boundingRect(*cSortedSquaresIter);

		// Reduce rect to 25% (50% per side) with center as anchor point
		cv::Point2i topLeft(rect.tl());
		cv::Point2i botRight(rect.br());
		cv::Point2i delta = botRight - topLeft;
		topLeft += (delta * 0.25);
		botRight -= (delta * 0.25);

		rect = cv::Rect(topLeft, botRight);

		// Define the Region of Interest (ROI)
		cv::Mat imageRect = image(rect);

		lowerMat = cv::Mat(imageRect.size(), CV_8UC3, colorRange[0]);
		upperMat = cv::Mat(imageRect.size(), CV_8UC3, colorRange[1]);
		resultMat = cv::Mat::zeros(imageRect.size(), CV_8U);

		cv::inRange(imageRect, lowerMat, upperMat, resultMat);

		vector<cv::Mat> planes;
		cv::split(imageRect, planes);
		for (int c=0; c<m_numOfColorPlanes; ++c)
		{
			resultMat.copyTo(planes[c]);
		}
		cv::merge(planes, imageRect);

		// For this algorithm, we make sure that we have 100% containment within the color range
		// 	For this we use a pixel-wise logical-AND operation
		int nr = resultMat.rows;
		int nc = resultMat.cols; // * resultMat.channels(); // Should be only one channel, but just in case...
		for (int i=0; i<nr && isInRange; ++i)
		{
			uchar* data = resultMat.ptr<uchar>(i);
			for (int j=0; j<nc && isInRange; ++j)
			{
				if (data[j] != 255)
				{
					isInRange = false;
				}
			}
		}

		if (isInRange)
		{
			sortedByRGBSquares.push_back(*cSortedSquaresIter);
		}
		else
		{
			isInRange = true;
		}
	}
}


// filters out squares found based on color, position, and size
void CRectangleDetector::FilterSquares(const cv::Mat& image,
		vector<vector<cv::Point> > colorSquares[],
		vector<vector<cv::Point> > &squares)
{
	squares.clear();

	const int maxsize = 100000;
	const int minDstBtnCtrs = 50;
	vector<vector<cv::Point> > filteredBySizeSquares[3];
	vector<vector<vector<cv::Point> > > sortedByCenterSquares[3];
	vector<vector<cv::Point> > consolidatedSquares;
	vector<vector<cv::Point> > sortedByRGBSquares;

	// Eliminate squares that are greater than maximum allowable size (min size has already been filtered out)
	FilterByMaxSize(colorSquares, filteredBySizeSquares, maxsize);

	// Sorts Squares into groups which share the same approximate center
	//	This is calculated based on the containing circle of the contours as well as the distance between centers
	SortByCenters(filteredBySizeSquares, sortedByCenterSquares, minDstBtnCtrs);

	// DumpSortedCenterData(sortedByCenterSquares);

	// Reduces the squares to a single, unique square across all color planes
	ConsolidateSquares(sortedByCenterSquares, consolidatedSquares);

	// This is a very permissive range being the whole upper-half of the gray-scale spectrum
	//		Later on the algorithm will reject any image with any pixels out of this range, so we need it to be permissive
	vector<cv::Scalar> colorRange;
	colorRange.push_back(cv::Scalar(125,125,125));
	colorRange.push_back(cv::Scalar(255,255,255));

	// Removes squares that do not fit within the prescribed color range
	FilterByBGR(image, consolidatedSquares, sortedByRGBSquares, colorRange);

//TODO: Add FilterByHSL()???   // This might come in useful as resistance to different lighting conditions

	for (int c=0; c<m_numOfColorPlanes; ++c)
	{
		// Make sure we don't need the old, unfiltered squares data here any more
		colorSquares[c].clear();
		copy(filteredBySizeSquares[c].begin(), filteredBySizeSquares[c].end(), back_inserter(colorSquares[c]));
	}

	copy(sortedByRGBSquares.begin(), sortedByRGBSquares.end(), back_inserter(squares));
}


// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void CRectangleDetector::FindSquares( const cv::Mat& image,
		vector<vector<cv::Point> >& squares,
		vector<vector<cv::Point> > colorSquares[] )
{
    squares.clear();

    cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    cv::pyrUp(pyr, timg, image.size());
    vector<vector<cv::Point> > contours;

    // find squares in every color plane of the image
    for ( int c=0; c<3; ++c )
    {
    	colorSquares[c].clear();

        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < m_numThreshIters; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cv::Canny(gray0, gray, 0, m_thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/m_numThreshIters;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<cv::Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
            	cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(cv::contourArea(cv::Mat(approx))) > 1000 &&
                    cv::isContourConvex(cv::Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        colorSquares[c].push_back(approx);
                }
            }
        }
    }

    FilterSquares(image, colorSquares, squares);
}


// the function draws all the squares in the image on their respective color planes and colors
cv::Mat CRectangleDetector::DrawSquares( cv::Mat& image, const vector<vector<cv::Point> > squares[] )
{
	cv::Scalar color;
	const cv::Scalar CV_BLUE(255, 0, 0);
	const cv::Scalar CV_GREEN(0, 255, 0);
	const cv::Scalar CV_RED(0, 0, 255);

	for (int c=0; c<m_numOfColorPlanes; ++c)
	{
		switch(c)
		{
		case 0:
			color = CV_BLUE;
			break;
		case 1:
			color = CV_GREEN;
			break;
		case 2:
			color = CV_RED;
			break;
		default:
			break;
		}

		for( size_t i = 0; i < squares[c].size(); i++ )
		{
			const cv::Point* p = &squares[c][i][0];
			int n = (int)squares[c][i].size();
			cv::polylines(image, &p, &n, 1, true, color, 3, CV_AA);
		}
	}

    return image;
}

// the function draws all the squares in the image
cv::Mat CRectangleDetector::DrawSquares( cv::Mat& image, const vector<vector<cv::Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const cv::Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 3, CV_AA);
    }

    return image;
}
