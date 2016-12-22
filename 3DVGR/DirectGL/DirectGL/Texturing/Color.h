#pragma once

#include <DirectGL/Math/Vector.h>

#include <vector>

namespace DirectGL
{
	namespace Texturing
	{
		template<typename T>
		class Color3 : public Math::Vector3<T>
		{
		public:
			using this_type = Color3;
			using base_type = Math::Vector3<T>;

			Color3() : base_type() {}
			template<typename U, typename... Us>
			Color3(U u, Us ...args) : base_type(u, args...) {}

			value_type r() const { return x(); }
			value_type g() const { return y(); }
			value_type b() const { return z(); }

			value_type &r() { return x(); }
			value_type &g() { return y(); }
			value_type &b() { return z(); }

			const Color3 rgb() const { return base_type::xyz(); }
			const Color3 rbg() const { return base_type::xzy(); }
			const Color3 grb() const { return base_type::yxz(); }
			const Color3 gbr() const { return base_type::yzx(); }
			const Color3 brg() const { return base_type::zxy(); }
			const Color3 bgr() const { return base_type::zyx(); }
			const Color3 rg() const  { return base_type::xy(); }
			const Color3 rb() const  { return base_type::xz(); }
			const Color3 gr() const  { return base_type::yx(); }
			const Color3 gb() const  { return base_type::yz(); }
			const Color3 br() const  { return base_type::zx(); }
			const Color3 bg() const  { return base_type::zy(); }

			this_type &operator=(const Color3 &other) { base_type::operator=(other); return *this; }
		};

		typedef Color3<GLfloat> Color3f;
		typedef Color3<GLubyte> Color3b;
		typedef std::vector<Color3f> ColorBuffer;
	}
}