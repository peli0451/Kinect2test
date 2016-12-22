#pragma once

#include <string>
#include <sstream>
#include <vector>

namespace DirectGL
{
	namespace Utility
	{
		namespace String
		{
			template<typename T>
			std::string toString(const T &value)
			{
				std::stringstream converter;	
				converter << value;
				return converter.str();
			}

			template<typename T, typename U>
			T string_cast(const U &value)
			{
				T retVal;
				retVal.assign(value.begin(), value.end());
				return retVal;
			}

			template <typename T>
			std::vector<T> split(const T &expression, const T &at)
			{
				std::vector<T> retVal;
				T::size_type start = 0, curr = 0;
				for (T::const_iterator it = expression.begin(), e = expression.end(); it != e; ++it)
				{
					bool isDelim = false;
					for (T::const_iterator ita = at.begin(), ea = at.end(); ita != ea; ++ita)
						isDelim |= (*it == *ita);

					if (isDelim)
					{
						if (start < curr)
							retVal.push_back(expression.substr(start, curr - start));
						start = ++curr;
					}
					else
						++curr;
				}
				if (start < curr)
					retVal.push_back(expression.substr(start, curr - start));
				return retVal;
			}
		}
	}
}

#define string_cast DirectGL::Utility::String::string_cast