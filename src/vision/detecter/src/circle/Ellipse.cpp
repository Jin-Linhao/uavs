
#include "Ellipse.h"

//Constructors
CEllipse::CEllipse(void)
{
}

CEllipse::CEllipse(float xc, float yc, float A, float B, float rho)
	:
	_xc(xc),
	_yc(yc),
	_A(A),
	_B(B),
	_rho(rho),
	_score(0.f),
	_rel(0.f)
{
}

CEllipse::CEllipse(const CEllipse& other)
	:
	_xc(other._xc),
	_yc(other._yc),
	_A(other._A),
	_B(other._B),
	_rho(other._rho),
	_score(other._score),
	_rel(other._rel)
{
}

CEllipse::CEllipse(float xc, float yc, float A, float B, float rho, float score, float rel)
	:
	_xc(xc),
	_yc(yc),
	_A(A),
	_B(B),
	_rho(rho),
	_score(score),
	_rel(rel)
{
}

CEllipse::~CEllipse(void)
{
}


//Used for sorting ellipses. 
bool CEllipse::operator<(const CEllipse& other) const
	{
		if(_score == other._score)
		{
			if(_rel == other._rel)
			{
				float lhs_e = _B / _A;
				float rhs_e = other._B / other._A;
				if(lhs_e == rhs_e)
				{
					return false;
				}
				return lhs_e > rhs_e;
			}
			return _rel > other._rel;
		}
		return _score > other._score;
	};


//Approximation of ellipse perimeter.
float CEllipse::GetPerimeter() const
{
	float h = ((_A - _B)*(_A - _B)) / ((_A + _B)*(_A + _B));
	float p = CV_PI * (_A + _B) * (1 + (0.25f*h) + (0.015625f*h*h) + (0.00390625f*h*h*h));
	return p;
}

