/** @file Types.h ===========================================================
*
*	Contains typedefs for math types used throughout the library.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <DirectGL/Texturing/Color.h>
#include <DirectGL/Math/Vector.h>

#include <Eigen/Eigen>
#include <epoxy/gl.h>

#include <vector>

namespace DirectGL
{
	using Matrix4f = Eigen::Matrix<GLfloat, 4, 4> ;
	using Matrix3f = Eigen::Matrix<GLfloat, 3, 3> ;
	using Rotation3f = Eigen::Quaternionf ;
	using Translation3f = Eigen::Translation3f ;
	using Scale3f = Eigen::AlignedScaling3f ;
	using Vector4f = Math::Vector4f;
	using Vector3f = Math::Vector3f;
	using Vector2f = Math::Vector2f;
	using Vector4i = Math::Vector4i;
	using Vector3i = Math::Vector3i;
	using Vector2i = Math::Vector2i;
	using Position2D = Vector2f;
	using Position3D = Vector3f;
	using Direction3D = Vector3f;
	using Index4D = Vector4i;
	using Index3D = Vector3i;
	using Index2D = Vector2i;
	using Color3f = Texturing::Color3f;
	using Color3b = Texturing::Color3b;
	using Buffer4Di = std::vector<Index4D>;
	using Buffer3Df = std::vector<Position3D>;
	using Buffer3Di = std::vector<Index3D>;
	using Buffer2Df = std::vector<Position2D>;
	using Buffer2Di = std::vector<Index2D>;
	using VertexBuffer = Buffer3Df;
	using NormalBuffer = Buffer3Df;
	using QuadBuffer = Buffer4Di;
	using TriangleBuffer = Buffer3Di;
	using TexCoordBuffer = Buffer2Df;
	using LineBuffer = Buffer2Di;
	using ColorBuffer = Texturing::ColorBuffer;

	struct Rect
	{
		Rect() : x(0), y(0), width(0), height(0) {}
		Rect(GLint x, GLint y, GLuint width, GLuint height) : x(x), y(y), width(width), height(height) {}
		GLint x,
			  y;
		GLuint width,
			   height;
	};
}