/** @file stdafx.h ============================================================
*
*	Include file for standard system include files, or project specific 
*	include files that are used frequently, but	are changed infrequently.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers

#define sgn(val) val < 0 ? -1 : 1

#define safe_delete(expr) delete expr; expr = 0
#define safe_delete_array(arr) delete[] arr; arr = 0

// APPLICATION HEADERS ========================================================

#include <epoxy/gl.h>
#ifdef WIN32
	#include <epoxy/wgl.h>
#else
	#include <epoxy/glx.h>
#endif

#include <string>
#include <ctime>
#include <fstream>
#include <vector>
#include <unordered_map>

#include <Eigen/Eigen>
#include <boost/smart_ptr/shared_ptr.hpp>