#pragma once

#define QGLFUNCTIONS_H
#define QOPENGL_H

#define NOMINMAX

#include <epoxy/gl.h>
#include <epoxy/wgl.h>
#include <GL/GL.h>

#include <DirectGL\DirectGL.h>

#include <QGLWidget>
#include <QObject>

#include <algorithm>
#include <unordered_map>
#include <list>

// only use vgr when it is present
#ifdef VGR
#include <VGR.h>
#endif

#define MOUSE_UNIT 0.00174532925199432957692369076849f

class GLWidget : public QGLWidget
{
	Q_OBJECT

public:
	GLWidget() : QGLWidget(), m_color(DirectGL::Texturing::Color3f(0.f, 0.3f, 0.5f)) { setupTimer(); }
	GLWidget(const QGLFormat & format, QWidget * parent = 0, GLWidget * shareWidget = 0, Qt::WindowFlags f = 0)
		: QGLWidget(format, parent, shareWidget, f), m_color(DirectGL::Texturing::Color3f(0.f, 0.3f, 0.5f)) {
		setupTimer();
		m_shared = shareWidget;
	}
	GLWidget(QGLContext *context, QWidget* parent = 0, GLWidget* shareWidget = 0, Qt::WindowFlags f = 0)
		: QGLWidget(context, parent, shareWidget, f), m_color(DirectGL::Texturing::Color3f(0.f, 0.3f, 0.5f)) {
		setupTimer();
		m_shared = shareWidget;
	}
	~GLWidget();

	DirectGL::Cameras::StereoCamera &getCamera() { return m_camera; }

	void pressKey(int key) { m_pressedKeys[key] = true; }
	void releaseKey(int key) { m_pressedKeys[key] = false; }

	void setBGColor(const DirectGL::Texturing::Color3f &color);
	void setReadBuffer(DirectGL::Buffers::Renderbuffer &buff);

	void loadModel(const std::string &model);

	void increaseScale(float amount = 0.01) { m_customScale += amount; }
	void decreaseScale(float amount = 0.01) { m_customScale -= amount; }
	void increaseSeparation() { m_separation += 0.001f; m_camera.setSeparation(m_separation); }
	void decreaseSeparation() { m_separation -= 0.001f; m_separation = std::max(0.f, m_separation); m_camera.setSeparation(m_separation); }
	void increaseCameraSpeed() { m_cameraSpeed += 0.1f; }
	void decreaseCameraSpeed() { m_cameraSpeed -= 0.1f; }

	float getScale() const { return m_customScale; }
	float getCameraSpeed() const { return m_cameraSpeed; }
	float getSeparation() const { return m_separation; }
	float getLodThresh() const { return m_lodThresh; }
	float getNear() const { return m_near; }
	float getFar() const { return m_far; }
	void setScale(float scale) { m_customScale = scale; }
	void setCameraSpeed(float speed) { m_cameraSpeed = speed; }
	void setSeparation(float separation) { m_separation = std::max(0.f, separation); m_camera.setSeparation(m_separation); }
	void setLodThresh(float thresh);
	void setNear(float n) { m_near = n; }
	void setFar(float f) { m_far = f; }

protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();

	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);
	void mousePressEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);

private:
	virtual void render() {}
	virtual void initialize() {}
	virtual void resize(int w, int y) {}
	void setupTimer();
	void eventLoop();
	void SelectObjectByPixel(int x, int y);
	void SelectLightByPixel(int x, int y);
	void handleCameraMovement();
	void handleObjectMovement(int x, int y);
	void handleLightMovement(int x, int y);

	DirectGL::Cameras::StereoCamera m_camera;
	DirectGL::Utility::PrecisionTimer m_timer;

	std::unordered_map<int, bool> m_pressedKeys;

	DirectGL::Rect m_screen;
	DirectGL::Texturing::Color3f m_color;

	DirectGL::Buffers::Framebuffer m_fbo;
	DirectGL::Buffers::Renderbuffer m_colorBuff,
									m_depthBuff,
									*m_extBuff;
	GLWidget *m_shared = nullptr;

	DirectGL::Geometry::OBJTriangleModel m_model;
	DirectGL::Shaders::Program m_program;
	DirectGL::Context::Ptr m_context;

	float m_cameraSpeed,
		  m_near,
		  m_far;

#ifdef VGR
	vgrDatabase m_database = VGR_INVALID_DATABASE;
#endif

	DirectGL::Matrix4f m_vgrFlip;
	float m_customScale,
		m_separation,
		m_lodThresh;

	bool m_vgr_available;
	bool use_special_controls;

	int special_control_state;
	bool m_resetMouse;

	QPoint m_oldMousePos;
	bool m_mouseCapturing = false;
};

