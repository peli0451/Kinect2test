#pragma once

#include <string>
#include <unordered_map>
#include <fstream>
#include <exception>

#include <boost/smart_ptr.hpp>

#include <DirectGL/Utility/SafeInit.h>

#define LOGNORMAL(message) DirectGL::Utility::Logging::log(DirectGL::Utility::Logging::Normal, message)
#define LOGWARNING(message) DirectGL::Utility::Logging::log(DirectGL::Utility::Logging::Warning, message)
#define LOGERROR(message) DirectGL::Utility::Logging::log(DirectGL::Utility::Logging::Error, message)
#define LOGEXCEPTION(message) {DirectGL::Utility::Logging::log(DirectGL::Utility::Logging::Error, message); throw std::exception(message);}

namespace DirectGL
{
	namespace Utility
	{
		class Logging
		{
		public:
			enum LogType
			{
				Normal,
				Warning,
				Error
			};

			class Loggable
			{
				friend class Logging;
			public:
				Loggable() { registerStream(*this); }
				virtual ~Loggable() { unregisterStream(*this); }

			private:
				virtual void log(const LogType &type, const std::string &message) = 0;
			};

			class File : public std::ofstream, public Loggable
			{
			public:
				using Ptr = boost::shared_ptr<File>;
				File() {}
				File(const std::string &path) : std::ofstream(path) {}

			private:
				void log(const Logging::LogType &type, const std::string &message);
			};
#ifdef _MSC_VER
			class VSDebug : public Loggable
			{
			public:
				using Ptr = boost::shared_ptr<VSDebug>;
			private:
				void log(const Logging::LogType &type, const std::string &message);
			};
#endif
			static void registerStream(Loggable &stream);
			static void unregisterStream(Loggable &stream);
			static void clearStreams() { m_streams->clear(); }

			static void log(const LogType &type, const std::string &message);
			static std::string getTimestamp();

		private:
			using StreamTable = std::unordered_map<Loggable*, char>;
			using StreamTable_safe = SafeInit<StreamTable>;
			static StreamTable_safe m_streams;
		};
	}
}