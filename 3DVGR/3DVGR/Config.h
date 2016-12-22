#pragma once

#include <string>
#include <boost/lexical_cast.hpp>
#include <unordered_map>

class Config
{
public:
	Config() {}
	Config(const std::string &path) { fromFile(path); }

	void fromFile(const std::string &path);
	void toFile(const std::string &path);

	template<typename T>
	inline bool getEntry(const std::string &entry, T &value);
	template<typename T>
	inline T getEntry(const std::string &entry);

	template<typename T>
	inline void setEntry(const std::string &entry, const T &value);

private:
	std::unordered_map<std::string, std::string> m_entries;
};

template<typename T>
bool Config::getEntry(const std::string &entry, T &value)
{
	auto found(m_entries.find(entry));
	if (found != m_entries.end())
	{
		value = boost::lexical_cast<T>(found->second);
		return true;
	}
	else
	{
		value = T();
		return false;
	}
}

template<typename T>
T Config::getEntry(const std::string &entry)
{
	T retVal;
	getEntry(entry, retVal);
	return retVal;
}

template<typename T>
void Config::setEntry(const std::string &entry, const T &value)
{
	m_entries[entry] = boost::lexical_cast<std::string>(value);
}