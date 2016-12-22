#pragma once

#include <epoxy/gl.h>
#include <GL/GLU.h>

#include "Logging.h"

#define OPENGL_LOG_WARN(err, message) \
		{ \
		const char *msg = reinterpret_cast<const char*>(gluErrorString(err)); \
		if (msg != nullptr) \
			LOGWARNING(message + std::string(" <") + std::string(msg) + ">"); \
		else \
			LOGWARNING(message + std::string(" <") + std::string("unknown error") + ">"); \
		}

#define OPENGL_LOG_ERR(err, message) \
		{ \
		const char *msg = reinterpret_cast<const char*>(gluErrorString(err)); \
		if (msg != nullptr) \
			LOGERROR(message + std::string(" <") + std::string(msg) + ">"); \
		else \
			LOGERROR(message + std::string(" <") + std::string("unknown error") + ">"); \
		}

#define OPENGL_ERROR_WARN(warnMsg) \
	{ \
		GLenum err = glGetError(); \
		if (err != GL_NO_ERROR) \
		{ \
			OPENGL_LOG_WARN(err, warnMsg); \
		} \
	}

#define OPENGL_ERROR_CHECK(errMsg) \
	{ \
		GLenum err = glGetError(); \
		if (err != GL_NO_ERROR) \
		{ \
			OPENGL_LOG_ERR(err, errMsg); \
		} \
	}