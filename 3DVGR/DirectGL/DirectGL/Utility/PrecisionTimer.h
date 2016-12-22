#pragma once

#include <Windows.h>

namespace DirectGL
{
	namespace Utility
	{
		class PrecisionTimer
		{
		public:
			enum Unit
			{
				hours,
				minutes,
				seconds,
				milliseconds,
				microseconds,
				nanoseconds
			};
			PrecisionTimer() { start(); m_unit = 1.f; }

			void start();
			float stop();
			float restart();

			void setUnit(float inSeconds) { m_unit = inSeconds; }
			void setUnit(const Unit &unit);

		private:
			LARGE_INTEGER m_start;
			LARGE_INTEGER m_freq;
			float m_unit;
		};
	}
}