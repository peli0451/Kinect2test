// ShaderCompiler.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <fstream>
#include <sstream>

int _tmain(int argc, _TCHAR* argv[])
{
	if (argc < 3)
		return 0;

	std::wstring fileName(argv[1]);
	std::wstringstream dummyLog;

	//initializing OpenGL using a dummy window/context
	HWND handle = CreateWindowW(L"Edit", L"void", 0, 0, 0, 0, 0, 0, NULL, GetModuleHandle(NULL), NULL);
	HDC dc = GetDC(handle);
	if (!SetPixelFormat(dc, 7, NULL))
	{
		OutputDebugString(L"Pixel format setting failed!\n");
		std::cout << "Pixel format setting failed!\n";
		dummyLog << "Pixel format setting failed!\n";

		std::flush(std::cout);
		ReleaseDC(handle, dc);
		DestroyWindow(handle);
		return -1;
	}

	HGLRC glContext = wglCreateContext(dc);
	if (!glContext)
	{
		OutputDebugString(L"GL context creation failed!\n");
		std::cout << "GL context creation failed!\n";
		dummyLog << "GL context creation failed!\n";
		std::flush(std::cout);
		ReleaseDC(handle, dc);
		DestroyWindow(handle);
		return -2;
	}
	wglMakeCurrent(dc, glContext);

	// setting up real GL Context
	const int attribs[] = { WGL_CONTEXT_MAJOR_VERSION_ARB, 3, WGL_CONTEXT_MINOR_VERSION_ARB, 3, 0 };
	auto newglContext = wglCreateContextAttribsARB(dc, NULL, attribs);

	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(glContext);
	//ReleaseDC(handle, dc);
	//DestroyWindow(handle);
	glContext = newglContext;
	
	if (!glContext)
	{
		OutputDebugString(L"Real GL context creation failed!\n");
		std::cout << "Real GL context creation failed!\n";
		dummyLog << "Real GL context creation failed!\n";
		std::flush(std::cout);
		ReleaseDC(handle, dc);
		DestroyWindow(handle);
		return -2;
	}
	wglMakeCurrent(dc, glContext);

	std::ifstream shaderFile(fileName);
	if (!shaderFile.fail())
	{
		shaderFile.seekg(0, std::ios_base::end);
		std::size_t fileSize = static_cast<std::size_t>((shaderFile.tellg()));
		GLchar *shaderCode = new GLchar[fileSize + 1];
		ZeroMemory(shaderCode, fileSize + 1);
		shaderFile.seekg(0, std::ios_base::beg);
		shaderFile.read(shaderCode, fileSize);
		shaderFile.close();

		GLuint shaderID;
		std::wstring shaderType(fileName.substr(fileName.find_last_of(L".") + 1));
		std::transform(shaderType.begin(), shaderType.end(), shaderType.begin(), ::tolower);
		if (shaderType == L"vert")
			shaderID = glCreateShader(GL_VERTEX_SHADER);
		else if (shaderType == L"frag")
			shaderID = glCreateShader(GL_FRAGMENT_SHADER);
		else if (shaderType == L"tesc")
			shaderID = glCreateShader(GL_TESS_CONTROL_SHADER);
		else if (shaderType == L"tese")
			shaderID = glCreateShader(GL_TESS_EVALUATION_SHADER);
		else if (shaderType == L"geom")
			shaderID = glCreateShader(GL_GEOMETRY_SHADER);
		else
		{
			OutputDebugString(L"Shader type not recognizable!\n");
			std::cout << "Shader type not recognizable!\n";
			dummyLog << "Shader type not recognizable!\n";
			std::flush(std::cout);
			return -5;
		}

		glShaderSource(shaderID, 1, &shaderCode, NULL);
		glCompileShader(shaderID);
		GLint status = 0;
		glGetShaderiv(shaderID, GL_COMPILE_STATUS, &status);

		auto dirPos(fileName.find_last_of(L"\\/"));
		std::string fn(fileName.begin() + (dirPos == std::wstring::npos ? 0 : dirPos + 1), fileName.end());
		std::cout << "<" << fn.c_str() << ">: ";
		if (status == GL_FALSE)
		{
			OutputDebugString(L"Encountered errors during compilation:\n");
			std::cout << "Encountered errors during compilation:\n";
			dummyLog << "Encountered errors during compilation:\n";
			std::flush(std::cout);
		}
		else
		{
			OutputDebugString(L"Shader successfully compiled\n");
			std::cout << "Shader successfully compiled\n";
			dummyLog << "Shader successfully compiled\n";
			std::flush(std::cout);

			std::wstring outDir(argv[2]);
			auto p = fileName.find_last_of(L"\\/");
			auto l = outDir + L"\\" + fileName.substr(p + 1) + L".slog";
			std::wofstream dummyLogFile(l, std::ios_base::trunc);
			dummyLogFile << dummyLog.str().c_str();
		}

		status = 0;
		glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &status);

		if (status > 0)
		{
			GLchar *errorLog = new GLchar[status];
			glGetShaderInfoLog(shaderID, status, &status, errorLog);
			OutputDebugStringA(errorLog);
			std::stringstream errStream(errorLog);
			errStream.seekg(0, std::ios_base::beg);
			while (!errStream.eof())
			{
				char buff[2048];
				ZeroMemory(buff, sizeof(char)* 2048);
				errStream.getline(buff, 2048);
				std::string line(buff);
				auto p = line.find_first_not_of("0123456789");
				p = line.find_first_of("0123456789", p);
				auto p2 = line.find_first_not_of("0123456789", p);
				std::string filename;
				filename.assign(fileName.begin(), fileName.end());
				if(p != std::string::npos) std::cout << filename + "(" + line.substr(p, p2 - p) + ")" + line.substr(p2 + 1) + "\n";
			}
			dummyLog << errorLog;
			delete[] errorLog;
		}

		glDeleteShader(shaderID);
		delete[] shaderCode;
	}

	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(glContext);
	ReleaseDC(handle, dc);
	DestroyWindow(handle);

	return 0;
}

