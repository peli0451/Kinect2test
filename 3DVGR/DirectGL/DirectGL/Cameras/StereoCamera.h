/** @file StereoCamera.h ======================================================
*
*	Gives a Stereo Camera Class extending the base Quaternion Camera to
*	calculate left and right eye matrices given an eye separation.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <DirectGL/Types.h>
#include <DirectGL/Cameras/QuaternionCamera.h>

namespace DirectGL
{
	namespace Cameras
	{
		class StereoCamera : public QuaternionCamera
		{
		public:
		// SETTERS ====================================================================
			inline void setSeparation(float distance) noexcept { m_separation = distance; }
			
		// GETTERS ====================================================================	
			inline float getSeparation() const noexcept { return m_separation; }

			DirectGL::Matrix4f getLeftMatrix() const noexcept;
			DirectGL::Matrix4f getRightMatrix() const noexcept;
			DirectGL::Matrix4f getLeftCameraMatrix() const noexcept;
			DirectGL::Matrix4f getRightCameraMatrix() const noexcept;

		private:
		// INTERNAL STATE =============================================================
			float m_separation;
		};
	}
}