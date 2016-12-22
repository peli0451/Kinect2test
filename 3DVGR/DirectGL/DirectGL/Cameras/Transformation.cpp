#include "stdafx.h"
#include "Transformation.h"

void DirectGL::Cameras::Transformation::translate(const Eigen::Translation3f &translation) noexcept
{
	m_translation.vector() += translation.vector();
}

void DirectGL::Cameras::Transformation::rotate(const Eigen::Quaternionf &rotation) noexcept
{
	m_rotation = rotation * m_rotation;
}

void DirectGL::Cameras::Transformation::scale(const Eigen::AlignedScaling3f &scaling) noexcept
{
	m_scale.diagonal().x() *= scaling.diagonal().x();
	m_scale.diagonal().y() *= scaling.diagonal().y();
	m_scale.diagonal().z() *= scaling.diagonal().z();
}

Eigen::Vector3f DirectGL::Cameras::Transformation::getViewDirection() const noexcept
{
	return m_rotation._transformVector(Eigen::Vector3f(0.f, 0.f, -1.f));
}

void DirectGL::Cameras::Transformation::translateLocal(const Eigen::Translation3f &translation) noexcept
{
	m_translation.vector() += m_rotation._transformVector(translation.vector());
}

void DirectGL::Cameras::Transformation::rotateX(float magnitude) noexcept
{
	m_rotation = Eigen::Quaternionf(Eigen::AngleAxisf(magnitude, Eigen::Vector3f(1.f, 0.f, 0.f))) * m_rotation;
}

void DirectGL::Cameras::Transformation::rotateY(float magnitude) noexcept
{
	m_rotation = Eigen::Quaternionf(Eigen::AngleAxisf(magnitude, Eigen::Vector3f(0.f, 1.f, 0.f))) * m_rotation;
}

void DirectGL::Cameras::Transformation::rotateZ(float magnitude) noexcept
{
	m_rotation = Eigen::Quaternionf(Eigen::AngleAxisf(magnitude, Eigen::Vector3f(0.f, 0.f, 1.f))) * m_rotation;
}

void DirectGL::Cameras::Transformation::rotateLocalX(float magnitude) noexcept
{
	m_rotation = m_rotation * Eigen::Quaternionf(Eigen::AngleAxisf(magnitude, Eigen::Vector3f(1.f, 0.f, 0.f)));
}

void DirectGL::Cameras::Transformation::rotateLocalY(float magnitude) noexcept
{
	m_rotation = m_rotation * Eigen::Quaternionf(Eigen::AngleAxisf(magnitude, Eigen::Vector3f(0.f, 1.f, 0.f)));
}

void DirectGL::Cameras::Transformation::rotateLocalZ(float magnitude) noexcept
{
	m_rotation = m_rotation * Eigen::Quaternionf(Eigen::AngleAxisf(magnitude, Eigen::Vector3f(0.f, 0.f, 1.f)));
}

Eigen::Matrix4f DirectGL::Cameras::Transformation::getMatrix() const noexcept
{
	return (m_translation * m_rotation * m_scale).matrix();
}

bool DirectGL::Cameras::Transformation::operator==(const Transformation & other) const noexcept
{
	bool retVal = other.m_rotation.isApprox(m_rotation, FLT_EPSILON) &&
				  other.m_scale.diagonal().isApprox(m_scale.diagonal(), FLT_EPSILON) &&
				  other.m_translation.isApprox(m_translation, FLT_EPSILON);
			
	return retVal;
}
