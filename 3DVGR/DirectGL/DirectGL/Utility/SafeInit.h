/** @file SafeInit.h ============================================================
*
*	Declares an utility template class for safe keeping of global variables by
*	providing a lazy, on demand initialization.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

namespace DirectGL
{
	namespace Utility
	{
		template <typename T>
		class SafeInit
		{
		public:
			inline static T &get() noexcept { static T object; return object; }
			inline T *operator->() const noexcept { return &get(); }
			inline T &operator*() const noexcept { return get(); }
		};
	}
}