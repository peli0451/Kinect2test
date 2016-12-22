#include "stdafx.h"
#include "Logging.h"

DirectGL::Utility::Logging::StreamTable_safe DirectGL::Utility::Logging::m_streams;

void DirectGL::Utility::Logging::registerStream(Loggable &stream)
{
	(*m_streams)[&stream] = 0;
}

void DirectGL::Utility::Logging::unregisterStream(Loggable &stream)
{
	StreamTable::iterator it = m_streams->find(&stream);
	if (it != m_streams->end())
		m_streams->erase(it);
}

void DirectGL::Utility::Logging::log(const LogType &type, const std::string &message)
{
	for (auto &stream : *m_streams)
	{
		stream.first->log(type, message);
	}
}

std::string DirectGL::Utility::Logging::getTimestamp()
{
	std::time_t time = std::time(nullptr);
	char buff[1024];
	std::strftime(buff, 1024, "%d-%m-%Y %H:%M:%S", localtime(&time));
	return std::string(buff);
}


void DirectGL::Utility::Logging::File::log(const Logging::LogType &type, const std::string &message)
{
	if (!is_open())
		throw std::exception("Trying to log to unopened LogFile!");

	std::string m = Logging::getTimestamp();
	switch (type)
	{
	case Logging::Normal:
		m += "           ";
		break;
	case Logging::Warning:
		m += " [WARNING] ";
		break;
	case Logging::Error:
		m += " [ERROR]   ";
		break;
	}

	m += message;
	m += "\n";
	write(m.c_str(), m.size());
	flush();
}

#ifdef _MSC_VER

#include <Windows.h>

void DirectGL::Utility::Logging::VSDebug::log(const Logging::LogType &type, const std::string &message)
{
	std::string m;
	switch (type)
	{
	case Logging::Normal:
		break;
	case Logging::Warning:
		m += "WARNING: ";
		break;
	case Logging::Error:
		m += "ERROR: ";
		break;
	}

	m += message;
	m += "\n";
	OutputDebugStringA(m.c_str());
}

#endif