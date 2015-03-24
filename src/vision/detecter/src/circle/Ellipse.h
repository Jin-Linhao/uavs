

/*
Data structure for an ellipse.

	float _xc		= center x-coordinate 
	float _yc		= center y-coordinate
	float _A		= major semi-axis length
	float _B		= minor semi-axis length
	float _rho		= orientation (in radians)
	float _score	= score, in interval [0,1]. Computed as formula [] in the paper.
	float _rel		= auxiliary measure of detection quality
*/


#pragma once

#include <opencv2/core/types_c.h>

class CEllipse
{

public:

	float _xc;
	float _yc;
	float _A;
	float _B;
	float _rho;
	float _score;
	float _rel;

	//Constructors
	CEllipse(void);
	CEllipse(const CEllipse& other);
	CEllipse(float xc, float yc, float A, float B, float rho);
	CEllipse(float xc, float yc, float A, float B, float rho, float score, float rel);
	~CEllipse(void);
	

	//Used for sorting ellipses. 
	bool operator<(const CEllipse& other) const;
	

	//Approximation of ellipse perimeter.
	float GetPerimeter() const;
};

