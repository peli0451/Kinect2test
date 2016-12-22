/** @file Transformation.h ==================================================
*
*	Declares the transformation class for encapsulating a basic combination
*	of a 3D translation, a three-axis rotation and a three-axis scaling.
*
*	@author Julian Meder
* =============================================================================
*/

#pragma once

#include <DirectGL/Types.h>

namespace DirectGL
{
	namespace Cameras
	{
		class Transformation
		{
		public:
		// CONSTRUCTORS ===============================================================
			inline Transformation() noexcept { reset(); }

		// SETTERS ====================================================================
			inline void setPosition(const Position3D &position) noexcept { m_translation.vector() = position; }
			inline void setTranslation(const Translation3f &translation) noexcept { m_translation = translation; }
			inline void setRotation(const Rotation3f &rotation) noexcept { m_rotation = rotation; }
			inline void setScaling(const Scale3f &scaling) noexcept { m_scale = scaling; }

		// GETTERS ===================================================================
			inline const auto &getPosition() const noexcept { return m_translation.vector(); }
			inline const Translation3f &getTranslation() const noexcept { return m_translation; }
			inline const Rotation3f &getRotation() const noexcept{ return m_rotation; }
			inline const Scale3f &getScaling() const noexcept { return m_scale; }
			Eigen::Vector3f getViewDirection() const noexcept;

			Eigen::Matrix4f getMatrix() const noexcept;

		// CONVENIENCE MANIPULATION METHODS ===========================================
			void translate(const Eigen::Translation3f &translation) noexcept;
			void translateLocal(const Eigen::Translation3f &translation) noexcept;
			void rotate(const Eigen::Quaternionf &rotation) noexcept;
			void scale(const Eigen::AlignedScaling3f &scaling) noexcept;

			void rotateX(float magnitude) noexcept;
			void rotateY(float magnitude) noexcept;
			void rotateZ(float magnitude) noexcept;

			void rotateLocalX(float magnitude) noexcept;
			void rotateLocalY(float magnitude) noexcept;
			void rotateLocalZ(float magnitude) noexcept;


		// RESETTERS ==================================================================
			inline void reset() noexcept { m_translation.vector().setZero(); m_rotation.setIdentity(); m_scale.setIdentity(); }

		// OPERATORS ==================================================================
			bool operator==(const Transformation &other) const noexcept;
			inline bool operator!=(const Transformation &other) const noexcept { return !operator==(other); }

		protected:
		// PASSED STATE ===============================================================
			Eigen::Translation3f m_translation;
			Eigen::Quaternionf m_rotation;
			Eigen::AlignedScaling3f m_scale;
		};
	}
}