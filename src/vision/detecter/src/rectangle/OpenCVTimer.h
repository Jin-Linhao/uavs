
#ifndef OPENCVTIMER_H_
#define OPENCVTIMER_H_

#include <iostream>
#include <vector>
#include <numeric>
using namespace std;

#include <opencv2/core/core.hpp>

class COpenCVTimer {
public:
	COpenCVTimer()
        : m_SizeOfBuffer(10),
		m_BufHeader(0),
		m_StartTicks(0),
		m_StopTicks(0),
		m_bIsTimerStarted(false),
		m_bIsBufferFull(false),
		m_bSuppressWarnings(false),
		m_RingBuffer(m_SizeOfBuffer)
	{
		initBuffer();
	}

	explicit COpenCVTimer(int _sizeOfBuffer)
		: m_SizeOfBuffer(_sizeOfBuffer),
		m_BufHeader(0),
		m_StartTicks(0),
		m_StopTicks(0),
		m_bIsTimerStarted(false),
		m_bIsBufferFull(false),
		m_bSuppressWarnings(false),
        m_RingBuffer(10)
	{
		initBuffer();
	}
	virtual ~COpenCVTimer() {}

	inline double startTimer();
	inline double stopTimer();
    inline void setSizeOfBuffer(int _sizeOfBuffer = 10);
	inline int getSizeOfBuffer() const { return m_SizeOfBuffer; }
	inline double getAverageTime() const;
	inline bool checkIsBufferFull() { return m_bIsBufferFull; }
	inline void setSuppressWarnings(bool suppress = false) { m_bSuppressWarnings = suppress; }

private: // methods

	void initBuffer(double val = 0.0);
	inline void setCurrentBufHeader(int _curBufHeader = 0);
	inline void clearRingBuffer() {  m_RingBuffer.clear(); }

private:
	int m_SizeOfBuffer;
	int m_BufHeader;
	double m_StartTicks;
	double m_StopTicks;
	bool m_bIsTimerStarted;
	bool m_bIsBufferFull;
	bool m_bSuppressWarnings;
	// Ring Buffer stores timing in sec not tickcounts
	// Ring buffers are inherently faster than shift registers
	// 		because the data does not constantly need to be shifted.
	vector<double> m_RingBuffer;
};

double COpenCVTimer::startTimer()
{
	m_bIsTimerStarted = true;
	return m_StartTicks = static_cast<double>(cv::getTickCount());
}

double COpenCVTimer::stopTimer()
{
	m_StopTicks = static_cast<double>(cv::getTickCount());
	if (m_bIsTimerStarted)
	{
		double delTime;
		double tickFrequency = cv::getTickFrequency();
		delTime = (m_StopTicks - m_StartTicks)/tickFrequency;

		m_RingBuffer.at(m_BufHeader) = delTime;

		if (m_BufHeader < (m_SizeOfBuffer-1))
		{
			++m_BufHeader;
		}
		else
		{
			m_BufHeader = 0;
			m_bIsBufferFull = true;
		}

		m_bIsTimerStarted = false;
	}
	else
	{
		if (!m_bSuppressWarnings)
		{
			cout << "No startTimer has been registered" << endl;
		}
	}
	return m_StopTicks;
}

void COpenCVTimer::setSizeOfBuffer(int _sizeOfBuffer /*10*/)
{
	if (m_SizeOfBuffer != _sizeOfBuffer)
	{
		m_SizeOfBuffer = _sizeOfBuffer;
		m_RingBuffer.clear();
		m_RingBuffer.resize(m_SizeOfBuffer);
		setCurrentBufHeader(0);
		m_bIsBufferFull = false;
	}

	if ((uint)m_SizeOfBuffer != m_RingBuffer.size())
	{
		m_RingBuffer.resize(m_SizeOfBuffer);
		m_RingBuffer.clear();
	}
}

double COpenCVTimer::getAverageTime() const
{
	if (m_bIsTimerStarted && !m_bSuppressWarnings)
	{
		cout << "COpenCVTimer has open startTimer value. Consider using stopTimer before calling getAverageTime" << endl;
	}

	double aveTime = 0.0;

	if (m_bIsBufferFull)
	{
		double sum_of_elements = accumulate(m_RingBuffer.begin(), m_RingBuffer.end(), 0.0);
		aveTime = sum_of_elements/static_cast<double>(m_SizeOfBuffer);
	}
	else
	{
		double sum_of_elements = accumulate(m_RingBuffer.begin(), m_RingBuffer.begin() + m_BufHeader, 0.0);
		aveTime = sum_of_elements/static_cast<double>(m_BufHeader);
		if (!m_bSuppressWarnings)
		{
			cout << "Average Time taken over only " << m_BufHeader << " data points" << endl;
		}
	}

	return aveTime;
}

void COpenCVTimer::initBuffer(double val /*0.0*/)
{
	setSizeOfBuffer(m_SizeOfBuffer);
	for (int i=0; i<m_SizeOfBuffer; ++i)
	{
		m_RingBuffer.at(i) = val;
	}
}

void COpenCVTimer::setCurrentBufHeader(int _curBufHeader /*0*/)
{
	if (_curBufHeader >= m_SizeOfBuffer)
	{
		cerr << "setCurrentBufHeader: _curBufHeader \"" << _curBufHeader <<
				"\" exceeds m_SizeOfBuffer: \"" << m_SizeOfBuffer << "\"" << endl;
	}
	else
	{
		m_BufHeader = _curBufHeader;
	}
}

#endif /* OPENCVTIMER_H_ */
