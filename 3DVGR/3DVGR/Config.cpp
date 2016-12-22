#include "Config.h"

#include <fstream>

void Config::fromFile(const std::string &path)
{
	std::ifstream f(path);
	if (!f.fail())
	{
		char buff[2048];
		buff[2047] = '\0';
		while (!f.eof())
		{
			f.getline(buff, 2047);
			std::string line(buff);

			auto pos1 = line.find_first_of('=');
			if (pos1 != std::string::npos)
			{
				int32_t i;
				for (i = pos1 - 1; i > 0 && line[i] == ' '; --i);
				auto entryName = line.substr(0, i + 1);
				if (entryName.size() > 0)
				{
					for (i = pos1 + 1; i < line.size() - 1 && line[i] == ' '; ++i);
					auto entryValue = line.substr(i);
					m_entries[entryName] = entryValue;
				}
			}
		}

		f.close();
	}
}

void Config::toFile(const std::string &path)
{
	std::ofstream f(path, std::ios_base::trunc);
	if (!f.fail())
		for (auto &entry : m_entries)
		{
			f << entry.first << " = " << entry.second << "\n";
		}
}