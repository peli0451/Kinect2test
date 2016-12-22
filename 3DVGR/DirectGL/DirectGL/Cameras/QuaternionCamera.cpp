#include "stdafx.h"
#include "QuaternionCamera.h"

#undef near
#undef far

DirectGL::Cameras::QuaternionCamera::QuaternionCamera(float FOV, float near, float far, float aspectRatio) noexcept
{
	reset();
	setProjective(FOV, near, far, aspectRatio);
}

DirectGL::Cameras::QuaternionCamera::QuaternionCamera(float left, float right, float top, float bottom, float near, float far) noexcept
{
	reset();
	setOrthogonal(left, right, top, bottom, near, far);
}

void DirectGL::Cameras::QuaternionCamera::setProjective(float FOV, float near, float far, float aspectRatio) noexcept
{
	float f  = 1.f / tan(FOV * static_cast<float>(M_PI) / 360.f),
		  nf = near - far;
	m_projection << f, 0, 0, 0,
					0, f * aspectRatio, 0, 0,
					0, 0, (near + far) / nf, 2 * near * far / nf,
					0, 0, -1, 0;
	m_FOV = FOV;
	m_near = near;
	m_far = far;
	m_aspect = aspectRatio;
}

void DirectGL::Cameras::QuaternionCamera::setOrthogonal(float left, float right, float top, float bottom, float near, float far) noexcept
{
	float rl = right - left,
		tb = top - bottom,
		fn = far - near;
	m_projection << 2.f / rl, 0, 0, (-right - left) / rl,
					0, 2.f / tb, 0, (-top - bottom) / tb,
					0, 0, -2.f / fn, (-far - near) / fn,
					0, 0, 0, 1;
	m_FOV = 0; // TODO calc from left/right, top/bottom
	m_near = near;
	m_far = far;
	m_aspect = abs(right - left) /  abs(top - bottom);
}

void DirectGL::Cameras::QuaternionCamera::forward(float magnitude) noexcept
{
	translateLocal(Eigen::Translation3f(0.f, 0.f, -magnitude));
}

void DirectGL::Cameras::QuaternionCamera::backward(float magnitude) noexcept
{
	translateLocal(Eigen::Translation3f(0.f, 0.f, magnitude));
}

void DirectGL::Cameras::QuaternionCamera::left(float magnitude) noexcept
{
	translateLocal(Eigen::Translation3f(-magnitude, 0.f, 0.f));
}

void DirectGL::Cameras::QuaternionCamera::right(float magnitude) noexcept
{
	translateLocal(Eigen::Translation3f(magnitude, 0.f, 0.f));
}

void DirectGL::Cameras::QuaternionCamera::up(float magnitude) noexcept
{
	translateLocal(Eigen::Translation3f(0.f, magnitude, 0.f));
}

void DirectGL::Cameras::QuaternionCamera::down(float magnitude) noexcept
{
	translateLocal(Eigen::Translation3f(0.f, -magnitude, 0.f));
}

void DirectGL::Cameras::QuaternionCamera::rotateUp(float magnitude) noexcept
{
	if (std::abs(m_xRot + magnitude) < M_PI / 2)
	{
		rotateLocalX(magnitude);
		m_xRot += magnitude;
	}
}

void DirectGL::Cameras::QuaternionCamera::rotateDown(float magnitude) noexcept
{
	if (std::abs(m_xRot - magnitude) < M_PI / 2)
	{
		rotateLocalX(-magnitude);
		m_xRot -= magnitude;
	}
}

void DirectGL::Cameras::QuaternionCamera::rotateRight(float magnitude) noexcept
{
	rotateY(-magnitude);
}

void DirectGL::Cameras::QuaternionCamera::rotateLeft(float magnitude) noexcept
{
	rotateY(magnitude);
}

Eigen::Matrix4f DirectGL::Cameras::QuaternionCamera::getMatrix() const noexcept
{
	return m_projection * (m_rotation.inverse() * m_translation.inverse()).matrix();
}

Eigen::Matrix4f DirectGL::Cameras::QuaternionCamera::getCameraMatrix() const noexcept
{
	return (m_translation * m_rotation).matrix();
}