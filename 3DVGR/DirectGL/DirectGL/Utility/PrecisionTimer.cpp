#include "stdafx.h"
#include "PrecisionTimer.h"

void DirectGL::Utility::PrecisionTimer::start()
{
	QueryPerformanceFrequency(&m_freq);
	QueryPerformanceCounter(&m_start);
}

float DirectGL::Utility::PrecisionTimer::stop()
{
	LARGE_INTEGER curr;
	QueryPerformanceCounter(&curr);

	return static_cast<float>(curr.QuadPart - m_start.QuadPart) / m_freq.QuadPart * m_unit;
}

float DirectGL::Utility::PrecisionTimer::restart()
{
	LARGE_INTEGER curr;
	QueryPerformanceCounter(&curr);

	float retVal = static_cast<float>(curr.QuadPart - m_start.QuadPart) / m_freq.QuadPart * m_unit;
	m_start = curr;
	QueryPerformanceFrequency(&m_freq);

	return retVal;
}

void DirectGL::Utility::PrecisionTimer::setUnit(const Unit &unit)
{
	switch (unit)
	{
	case Unit::hours:
		m_unit = 1.f / 60.f / 60.f;
		break;
	case Unit::minutes:
		m_unit = 1.f / 60.f;
		break;
	case Unit::seconds:
		m_unit = 1.f;
		break;
	case Unit::milliseconds:
		m_unit = 1000.f;
		break;
	case Unit::microseconds:
		m_unit = 1000000.f;
		break;
	case Unit::nanoseconds:
		m_unit = 1000000000.f;
		break;
	}
}