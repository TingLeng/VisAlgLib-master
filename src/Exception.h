#pragma once
#include <exception>
#include <string>

namespace VisionLibrary
{
	class Exception : std::exception
	{
	public:
		Exception(const std::string& _err, const std::string& _file, int _line)
			:err(_err), file(_file), line(_line)
		{
			char buf[256];
			sprintf_s(buf, "Error: %s in file: %s(line %d)", err.c_str(), file.c_str(), line);
			msg = std::string(buf);
		}

		virtual ~Exception() throw() {}

		virtual const char *what() const throw()
		{
			return msg.c_str();
		}

	private:
		std::string msg;
		std::string err;
		std::string file;
		int line;
	};
}