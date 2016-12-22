/** @file QuaternionCamera.h ==================================================
*
*	Provides the QuaternionCamera class for easy creation and manipulation of 
*	a free roaming camera and gives comfortable access to the necessary 
*	matrices for rendering.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <DirectGL/Cameras/Transformation.h>

#undef near
#undef far

namespace DirectGL
{
	namespace Cameras
	{
		class QuaternionCamera : public Transformation
		{
		public:
		// CONSTRUCTORS ===============================================================
			inline QuaternionCamera() noexcept { reset();  }
			QuaternionCamera(float FOV, float near, float far, float aspectRatio) noexcept;
			QuaternionCamera(float left, float right, float top, float bottom, float near, float far) noexcept;

		// PROJECTION INITIALIZERS ====================================================
			void setProjective(float FOV, float near, float far, float aspectRatio) noexcept;
			void setOrthogonal(float left, float right, float top, float bottom, float near, float far) noexcept;

		// MANIPULATION CONVENIENCE METHODS ===========================================
			void forward(float magnitude) noexcept;
			void backward(float magnitude) noexcept;
			void left(float magnitude) noexcept;
			void right(float magnitude) noexcept;
			void up(float magnitude) noexcept;
			void down(float magnitude) noexcept;

			void rotateLeft(float magnitude) noexcept;
			void rotateRight(float magnitude) noexcept;
			void rotateUp(float magnitude) noexcept;
			void rotateDown(float magnitude) noexcept;

		// SETTERS ====================================================================
			inline void setProjection(const Eigen::Matrix4f &projection)  noexcept { m_projection = projection; }
			inline const Eigen::Matrix4f &getProjection() const noexcept { return m_projection; }
			Eigen::Matrix4f getCameraMatrix() const noexcept;

		// GETTERS ====================================================================
			inline float getFOV() const noexcept { return m_FOV; }
			inline float getNearPlane() const noexcept { return m_near; }
			inline float getFarPlane() const noexcept { return m_far; }
			inline float getAspectRatio() const noexcept { return m_aspect; }

			Eigen::Matrix4f getMatrix() const noexcept;

		// RESETTERS ==================================================================
			inline void reset() noexcept { Transformation::reset(); m_projection.setIdentity(); m_xRot = 0; }
			inline void resetTransformation() noexcept { Transformation::reset(); m_xRot = 0; }

		private:
		// INTERNAL STATE =============================================================
			Eigen::Matrix4f m_projection;
			float m_xRot,
				  m_FOV,
				  m_near,
				  m_far,
				  m_aspect;
		};
	}
}