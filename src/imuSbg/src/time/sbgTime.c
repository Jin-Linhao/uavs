#include "sbgTime.h"

//------------------------------------------------------------------------------//
//- WIN32 Time functions                                                       -//
//------------------------------------------------------------------------------//
#ifdef WIN32
#include <time.h>
#include "windows.h"

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
uint32 sbgGetTime(void)
{
	return GetTickCount();
}

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
void sbgSleep(uint32 ms)
{
	Sleep(ms);
}

#else

#ifdef __APPLE__
#include <sys/time.h>
#include <unistd.h>
#else
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#endif

//------------------------------------------------------------------------------//
//- UNIX Time functions                                                        -//
//------------------------------------------------------------------------------//

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
uint32 sbgGetTime(void)
{
	clock_t clk;
	struct timeval tv;
	struct timezone tz;
	
	gettimeofday(&tv, &tz);
	clk = tv.tv_sec * 1000 + (tv.tv_usec / 1000);
	return clk;
}

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
void sbgSleep(uint32 ms)
{
	usleep(ms*1000);
}
#endif
