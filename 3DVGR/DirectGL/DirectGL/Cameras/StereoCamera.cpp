#include "stdafx.h"
#include "StereoCamera.h"

DirectGL::Matrix4f DirectGL::Cameras::StereoCamera::getLeftMatrix() const noexcept
{
	DirectGL::Translation3f leftTrans(m_separation, 0.f, 0.f);
	return getProjection() * (leftTrans * m_rotation.inverse() * m_translation.inverse()).matrix();
}

DirectGL::Matrix4f DirectGL::Cameras::StereoCamera::getRightMatrix() const noexcept
{
	DirectGL::Translation3f rightTrans(-m_separation, 0.f, 0.f);
	return getProjection() * (rightTrans * m_rotation.inverse() * m_translation.inverse()).matrix();
}

DirectGL::Matrix4f DirectGL::Cameras::StereoCamera::getLeftCameraMatrix() const noexcept
{
	DirectGL::Translation3f leftTrans(-m_separation, 0.f, 0.f);
	return (m_translation * m_rotation * leftTrans).matrix();
}

DirectGL::Matrix4f DirectGL::Cameras::StereoCamera::getRightCameraMatrix() const noexcept
{
	DirectGL::Translation3f rightTrans(m_separation, 0.f, 0.f);
	return (m_translation * m_rotation * rightTrans).matrix();
}