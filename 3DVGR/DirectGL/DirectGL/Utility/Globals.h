#pragma once

#include <DirectGL/Utility/Logging.h>

#include <string>
#include <sstream>
#include <unordered_map>

#include <typeinfo>

namespace DirectGL
{
	namespace Utility
	{
		class Globals
		{
		public:
			template<typename U>
			static bool set(const std::string &name, const U &value)
			{
				GlobalMap::iterator it = g_GlobalMap->find(name);
				if (it == g_GlobalMap->end())
				{
					return false;
				}
				else
				{
					Global<U> *ptr = dynamic_cast<Global<U>*>(it->second);
					if (ptr)
						ptr->set(value);
					else
					{
						std::string errmsg("Wrong type <" + std::string(typeid(U).name()) + "> for global variable \"" + name + "\" !");
						LOGERROR(errmsg);
						return false;
					}
					return true;
				}
			}
			template<>
			static bool set(const std::string &name, const std::string &value)
			{
				GlobalMap::iterator it = g_GlobalMap->find(name);
				if (it == g_GlobalMap->end())
				{
					return false;
				}
				else
				{
					it->second->setString(value);
					return true;
				}
			}

			template <typename U>
			static bool get(const std::string &name, U &value)
			{
				GlobalMap::iterator it = g_GlobalMap->find(name);
				if (it == g_GlobalMap->end())
				{
					return false;
				}
				else
				{
					Global<U> *ptr = dynamic_cast<Global<U>*>(it->second);
					if (ptr)
						value = ptr->get<U>();
					else
					{
						std::string errmsg("Wrong type <" + std::string(typeid(U).name()) + "> for global variable \"" + name + "\" !");
						LOGERROR(errmsg);
						return false;
					}
					return true;
				}
			}
			template <>
			static bool get(const std::string &name, std::string &value)
			{
				GlobalMap::iterator it = g_GlobalMap->find(name);
				if (it == g_GlobalMap->end())
				{
					return false;
				}
				else
				{
					value = it->second->getString();
					return true;
				}
			}

		protected:
			typedef std::unordered_map<std::string, Globals*> GlobalMap;
			static GlobalMap *g_GlobalMap;

		private:
			virtual void setString(const std::string &value) = 0;
			virtual std::string getString() const = 0;
		};

		template<typename T>
		class Global : public Globals
		{
		public:
			using value_type = T;

			Global(const std::string &name, const value_type &initalValue = value_type());
			~Global();

			inline const value_type &get() const noexcept { return m_value; }
			template<typename U>
			inline U get() const
			{
				std::stringstream reader;
				reader << m_value;
				reader.clear();
				reader.seekg(0, std::ios_base::beg);
				U parsed;
				reader >> parsed;
				return parsed;
			}

			inline void set(const value_type &value) noexcept { m_value = value; }
			template<typename U>
			inline void set(const U &value) noexcept
			{
				std::stringstream writer;
				writer << value;
				writer.clear();
				writer.seekg(0, std::ios_base::beg);
				writer >> m_value;
			}

			template<typename U>
			inline operator U() const noexcept { return get<U>(); }
			inline operator const value_type&() const  noexcept{ return get(); }
			
			template<typename U>
			inline void operator =(const U &value) noexcept { set<U>(value); }
			inline void operator =(const value_type &value) noexcept { set(value); }

		private:
			virtual void setString(const std::string &value) noexcept { set(value); }
			virtual std::string getString() const noexcept { return get<std::string>(); }

			std::string m_name;
			value_type m_value;

		};

		template<typename T>
		Global<T>::Global(const std::string &name, const value_type &initialValue)
		{
			if (name.empty())
			{
				LOGERROR("Global variable name must not be empty!");
				return;
			}

			if (g_GlobalMap)
			{
				GlobalMap::iterator it = g_GlobalMap->find(name);
				if (it != g_GlobalMap->end())
				{
					LOGWARNING("Global variable of name \"" + name + "\" already exists!");
					return;
				}
			}
			else
				g_GlobalMap = new GlobalMap();

			m_name = name;
			m_value = initialValue;
			(*g_GlobalMap)[name] = this;
		}

		template<typename T>
		Global<T>::~Global()
		{
			g_GlobalMap->erase(m_name);
			if (g_GlobalMap->empty())
			{
				delete g_GlobalMap;
				g_GlobalMap = nullptr;
			}
		}
	}
}