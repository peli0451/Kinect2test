/** @file Vector.h ===========================================================
*
*	Declares the DirectGL Vector class used throughout the library. The class
*	is basically a decorator for the Eigen vectors, just adding a few more
*	initializers and swizzle functions.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <Eigen/Dense>

namespace DirectGL
{
	namespace Math
	{

// HELPER DEFINES =============================================================
#define BASE_DECLARES(Class, Dim) using base_type = Eigen::Matrix<T, Dim, 1>; \
							using this_type = Class; \
							using value_type = T; \
							constexpr static const unsigned dim = Dim; \
							inline Class() : base_type() {} \
							template<typename U, typename... Us>		\
							inline Class(U u, Us ...args) : base_type(u, args...) {} \
							inline Class(const T *data) { memcpy(this, data, sizeof(Class)); } \
							template<typename U> \
							inline Class(const Class<U> &other) { *this = other.cast<T>(); } \
							template<typename U> \
							inline Class(const U *data) { *this = Class<U>(data).cast<T>(); } \
							inline T length() const { return base_type::norm(); } \
							inline this_type &operator=(const Class &other) { base_type::operator=(other); return *this; }

#define GEN_SWIZZLE_2(x, y) inline Vector2<T> x##y() const noexcept { return Vector2<T>(x(), y()); }
#define GEN_2_PERM(x, y) GEN_SWIZZLE_2(x, y) \
						 GEN_SWIZZLE_2(y, x)

#define GEN_SWIZZLE_3(x, y, z) inline Vector3<T> x##y##z() const noexcept { return Vector3<T>(x(), y(), z()); }
#define GEN_2_PERM_3(x, y, z)	GEN_SWIZZLE_3(x, y, z) \
								GEN_SWIZZLE_3(x, z, y)
#define GEN_3_PERM_3(x, y, z)	GEN_2_PERM_3(x, y, z) \
								GEN_2_PERM_3(y, x, z) \
								GEN_2_PERM_3(z, x, y) 
#define GEN_3_PERM(x, y, z)		GEN_3_PERM_3(x, y, z) \
								GEN_2_PERM(x, y) \
								GEN_2_PERM(x, z) \
								GEN_2_PERM(y, z)

#define GEN_SWIZZLE_4(x, y, z, w) inline Vector4 x##y##z##w() const noexcept { return Vector4(x##(), y##(), z##(), w##()); }
#define GEN_2_PERM_4(x, y, z, w)	GEN_SWIZZLE_4(x, y, z, w) \
									GEN_SWIZZLE_4(x, y, w, z)
#define GEN_3_PERM_4(x, y, z, w)	GEN_2_PERM_4(x, y, z, w) \
									GEN_2_PERM_4(x, z, y, w) \
									GEN_2_PERM_4(x, w, y, z) 
#define GEN_4_PERM(x, y, z, w) GEN_3_PERM_4(x, y, z, w) \
								 GEN_3_PERM_4(y, x, z, w) \
								 GEN_3_PERM_4(z, x, y, w) \
								 GEN_3_PERM_4(w, x, y, z) \
								 GEN_3_PERM_3(x, y, z) \
								 GEN_3_PERM_3(y, z, w) \
								 GEN_3_PERM_3(x, y, w) \
								 GEN_3_PERM_3(x, z, w) \
								 GEN_2_PERM(x, y) \
								 GEN_2_PERM(x, z) \
								 GEN_2_PERM(x, w) \
								 GEN_2_PERM(y, z) \
								 GEN_2_PERM(y, w) \
								 GEN_2_PERM(z, w) 

		template<typename T>
		class Vector2 : public Eigen::Matrix<T, 2, 1>
		{
		public:
			BASE_DECLARES(Vector2, 2)

		// CONSTRUCTORS ===============================================================
			inline Vector2(const T &x, const T &y) : base_type(x, y) {}
			
		// SWIZZLES ===================================================================
			GEN_2_PERM(x, y)
		};

		template<typename T>
		class Vector3 : public Eigen::Matrix<T, 3, 1>
		{
		public:
			BASE_DECLARES(Vector3, 3)
		
		// CONSTRUCTORS ===============================================================
			inline Vector3(const T &x, const T &y, const T &z) : base_type(x, y, z) {}
			inline Vector3(const Vector2<T> &xy, const T&z) : base_type(xy.x(), xy.y(), z) {}
			inline Vector3(const  T &x, const Vector2<T> &yz) : base_type(x, yz.y(), yz.z()) {}
			
		// SWIZZLES ===================================================================
			GEN_3_PERM(x, y, z)
		};

		template<typename T>
		class Vector4 : public Eigen::Matrix<T, 4, 1>
		{
		public:
			BASE_DECLARES(Vector4, 4)

		// CONSTRUCTORS ===============================================================
			inline Vector4(const T &x, const T &y, const T &z, const T &w) : base_type(x, y, z, w) {}
			inline Vector4(const Vector2<T> &xy, const T&z, const T &w) : base_type(xy.x(), xy.y(), z, w) {}
			inline Vector4(const T &x, const Vector2<T> &yz, const T &w) : base_type(x, yz.y(), yz.z(), w) {}
			inline Vector4(const T &x, const T &y, const Vector2<T> &zw) : base_type(x, y, zw.z(), zw.w()) {}
			inline Vector4(const Vector2<T> &xy, const Vector2<T> &zw) : base_type(xy.x(), xy.y(), zw.z(), zw.w()) {}
			inline Vector4(const Vector3<T> &xyz, const T &w) : base_type(xyz.x(), xyz.y(), xyz.z(), w) {}
			inline Vector4(const T &x, const Vector3<T> &yzw) : base_type(x, yzw.y(), yzw.z(), yzw.w()) {}

		// SWIZZLES ===================================================================
			GEN_4_PERM(x, y, z, w)
		};

#undef GEN_SWIZZLE_2
#undef GEN_SWIZZLE_3
#undef GEN_SWIZZLE_4
#undef GEN_2_PERM
#undef GEN_3_PERM
#undef GEN_4_PERM
#undef GEN_2_PERM_3
#undef GEN_2_PERM_4
#undef GEN_3_PERM_3
#undef GEN_3_PERM_4
#undef GEN_4_PERM
#undef BASE_DECLARES

// VECTOR TYPEDEFS ============================================================
#define DECLARE_VECTORS(type, suff) using Vector2##suff = Vector2<type>; \
									using Vector3##suff = Vector3<type>; \
									using Vector4##suff = Vector4<type>; 

		DECLARE_VECTORS(float, f)
		DECLARE_VECTORS(double, d)
		DECLARE_VECTORS(int, i)
		DECLARE_VECTORS(unsigned, u)

#undef DECLARE_VECTORS
	}
}
