#include "GLWidget.h"
#include <fstream>
#include <assert.h>
#include <QKeyEvent>
#include <QTimer>
#include <QFileDialog>

#include "TestTriangle.h"
#include "Config.h"

const char *key = "";

GLWidget::~GLWidget()
{
#ifdef VGR
	if (m_shared)
	{
		vgrCloseDatabase();
		vgrShutdownRenderer();
		vgrShutdownInterface();
	}
#endif
}

void GLWidget::initializeGL()
{

	m_near = 0.1;
	m_far = 1000.f;
	m_camera.setPosition(DirectGL::Vector3f(1000, 1000, 1000));
	setMouseTracking(true);
	use_special_controls = false;
	m_vgr_available = false;
	special_control_state = 0;
	m_context = DirectGL::Context::getCurrent();

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	m_screen.width = viewport[2] - viewport[0];
	m_screen.height = viewport[3] - viewport[1];

	if (m_shared)
	{
		kinectControl.init(this);

		m_shared->setReadBuffer(m_colorBuff);

		m_camera.setProjective(90.f, m_near, m_far, 4.f / 3.f);
		m_camera.setPosition(DirectGL::Vector3f(0.f, 0.2f, 0.8f));

		m_colorBuff.RenderbufferStorage(GL_RGBA, m_screen.width, m_screen.height);
		m_depthBuff.RenderbufferStorage(GL_DEPTH_COMPONENT, m_screen.width, m_screen.height);
		m_fbo.attach(GL_COLOR_ATTACHMENT0, m_colorBuff);
		m_program.fromFiles("simple.vert", "fragment.frag");

#ifdef VGR
		// initializing vgr
		vgrSetTraceLogfile(vgr_tracelevel_information, "vgrlogfile.txt");
		vgrAttributes initAttribs = vgrCreateAttributes(vgr_class_interfaceinit);
		vgrSetPropertys(initAttribs, vgr_interfaceinit_licence_path, "", 0);
		vgrSetPropertyi(initAttribs, vgr_interfaceinit_cache_mem_size, 1024);
		
		if (vgrInitializeInterface(key, initAttribs) != vgr_result_ok)
		{
			
		}
		else
		{
			vgrDestroyAttributes(initAttribs);
			initAttribs = vgrCreateAttributes(vgr_class_rendererinit);
			vgrSetPropertyi(initAttribs, vgr_rendererinit_vram_size, 512);

			if (vgrInitializeRenderer(initAttribs) != vgr_result_ok)
				throw std::exception("Error on vgr renderer initialization!");
			vgrDestroyAttributes(initAttribs);

			initAttribs = vgrGetRenderer();
			vgrSetPropertyf(initAttribs, vgr_renderer_lod_threshold, 1.f);
			vgrSetLightState(0, vgr_flag_set);
			vgrAttributes l = vgrCreateAttributes(vgr_class_light);
			vgrGetLight(0, l);
			float lightdiff[] = { 1.f, 1.f, 1.f, 1.f },
				lightspec[] = { 0.5f, 0.5f, 0.5f, 1.f },
				lightamb[] = { 0.2f, 0.2f, 0.2f, 1.f };

			vgrSetPropertyfv(l, vgr_light_specular, lightspec);
			vgrSetPropertyfv(l, vgr_light_ambient, lightamb);
			vgrSetPropertyfv(l, vgr_light_diffuse, lightdiff);
			vgrSetLight(0, l);
			vgrDestroyAttributes(l);

			vgrSetViewport(0, 0, m_screen.width, m_screen.height);

			m_vgrFlip << 1, 0, 0, 0,
				0, 0, 1, 0,
				0, -1, 0, 0,
				0, 0, 0, 1;	

			m_vgr_available = true;
		}
#endif
	}


	m_customScale = 1.f;
	m_separation = 0.02f;
	m_cameraSpeed = 1.f;
	m_camera.setSeparation(0.02f);
	setLodThresh(0.1f);
	OPENGL_ERROR_CHECK("GLWidget initialisation failed with error");
}

void GLWidget::setLodThresh(float thresh)
{
	m_lodThresh = thresh;
#ifdef VGR
	if (m_vgr_available)
	{
		auto initAttribs = vgrGetRenderer();
		vgrSetPropertyf(initAttribs, vgr_renderer_lod_threshold, thresh);
	}
#endif
}

void GLWidget::paintGL()
{
	glClearColor(m_color.r(), m_color.g(), m_color.b(), 1.f);
	DirectGL::Buffers::Framebuffer::bindWindowFramebuffer(GL_FRAMEBUFFER);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	DirectGL::ScopedEnable d(GL_DEPTH_TEST);

	if (m_extBuff)
	{
		m_fbo.attach(GL_COLOR_ATTACHMENT0, *m_extBuff);
		m_fbo.BlitToWindowFramebuffer(0, 0, m_screen.width, m_screen.height, 0, 0, m_screen.width, m_screen.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
	}
	else
	{
		eventLoop();
#ifdef VGR
		if (m_database)
		{
			vgrAttributes l = vgrCreateAttributes(vgr_class_light);
			vgrGetLight(0, l);
			float lightPos[4];
			auto campos(m_camera.getPosition());
			DirectGL::Vector4f m = m_vgrFlip.inverse() * DirectGL::Vector4f(campos.x(), campos.y(), campos.z(), 1);
			std::memcpy(lightPos, m.data(), sizeof(float)* 3);
			lightPos[3] = 1.f;
			vgrSetPropertyfv(l, vgr_light_position, lightPos);
			vgrSetLight(0, l);
			vgrDestroyAttributes(l);

			vgrSetCameraProjectionMatrix(m_camera.getProjection().data(), vgr_false);

			// rendering the left camera for the shared window first so we can copy it to the share framebuffer
			DirectGL::Matrix4f cScale;
			cScale << m_customScale, 0, 0, 0,
				0, m_customScale, 0, 0,
				0, 0, m_customScale, 0,
				0, 0, 0, 1;
			DirectGL::Matrix4f nMat = cScale;
			vgrSetDatabaseMatrix(m_database, nMat.data());
			DirectGL::Matrix4f camMat = m_vgrFlip.inverse() * m_camera.getLeftCameraMatrix();
			vgrSetCameraMatrix(camMat.data());

			vgrBeginFrame();
			vgrRender(vgr_render_all);
			vgrEndFrame();

			m_fbo.bind(GL_DRAW_FRAMEBUFFER);
			glBlitFramebuffer(0, 0, m_screen.width, m_screen.height, 0, 0, m_screen.width, m_screen.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

			DirectGL::Buffers::Framebuffer::bindWindowFramebuffer();
			camMat = m_vgrFlip.inverse() * m_camera.getRightCameraMatrix();
			vgrSetCameraMatrix(camMat.data());
			
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			vgrBeginFrame();
			glDisable(GL_CULL_FACE);
			vgrRender(vgr_render_all);
			vgrEndFrame();
		}
		else
		{
#endif
			m_model.getTransformation().setScaling(DirectGL::Scale3f(m_customScale, m_customScale, m_customScale));
			DirectGL::Shaders::Program::ScopedUse p(m_program);
			m_program.Uniform3fv("camPos", 1, m_camera.getPosition().data());
			m_program.UniformMatrix4fv("projection", 1, GL_FALSE, m_camera.getRightMatrix().data());
			m_program.UniformMatrix4fv("modelT", 1, GL_FALSE, m_model.getTransformation().getMatrix().data());
			m_model.draw(m_program);

			m_fbo.bind(GL_FRAMEBUFFER);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			m_program.UniformMatrix4fv("projection", 1, GL_FALSE, m_camera.getLeftMatrix().data());
			m_model.draw(m_program);
#ifdef VGR
		}
#endif
	}
	
	OPENGL_ERROR_CHECK("Error during render loop: ");
}

void GLWidget::resizeGL(int w, int h)
{
	glViewport(0, 0, w, h);
	m_camera.setProjective(90.f, 0.1f, 1000.f, static_cast<float>(w) / h);
	m_screen.width = w;
	m_screen.height = h;
	if (m_shared)
	{

#ifdef VGR
		vgrSetViewport(0, 0, m_screen.width, m_screen.height);
#endif

		m_colorBuff.RenderbufferStorage(GL_RGBA, m_screen.width, m_screen.height);
		m_depthBuff.RenderbufferStorage(GL_DEPTH_COMPONENT, m_screen.width, m_screen.height);

		m_fbo.attach(GL_COLOR_ATTACHMENT0, m_colorBuff);
		m_fbo.attach(GL_DEPTH_ATTACHMENT, m_depthBuff);
	}
}

void GLWidget::eventLoop()
{
	MotionParameters motionParameters = kinectControl.run();
	
	// TODO speed configurable
	float time = m_timer.restart() * m_cameraSpeed;
	DirectGL::Vector3f dir(DirectGL::Vector3f::Zero());

	float trans_strength = time*100;
	float rot_strength = time*50;

	//if (m_pressedKeys[Qt::Key::Key_X])
		dir += trans_strength * m_camera.getRotation()._transformVector(DirectGL::Vector3f(
			motionParameters.getTranslateX(),
			motionParameters.getTranslateY(),
			motionParameters.getTranslateZ())
		);
		Eigen::Quaternionf rot = Eigen::Quaternionf::Identity().slerp(rot_strength,motionParameters.getRotation());

	//Debug-Ausgabe

	if (m_pressedKeys[Qt::Key::Key_W])
		dir += m_camera.getRotation()._transformVector(DirectGL::Vector3f(0.f, 0.f, -time));
	if (m_pressedKeys[Qt::Key::Key_S])
		dir += m_camera.getRotation()._transformVector(DirectGL::Vector3f(0.f, 0.f, time));
	if (m_pressedKeys[Qt::Key::Key_D])
		dir += m_camera.getRotation()._transformVector(DirectGL::Vector3f(time, 0.f, 0.f));
	if (m_pressedKeys[Qt::Key::Key_A])
		dir += m_camera.getRotation()._transformVector(DirectGL::Vector3f(-time, 0.f, 0.f));

	if (motionParameters.getTarget() == MotionParameters::MotionTarget::TARGET_CAMERA) {
		m_camera.translate(DirectGL::Translation3f(dir));
		m_camera.rotateLocal(motionParameters.getRotation());
	}
	else {
		picked_model->getTransformation().translate(DirectGL::Translation3f(dir)); //ist "dir" vernünftig für Models?
		picked_model->getTransformation().rotateLocal(rot);
	}
	if (m_mouseCapturing || use_special_controls)
	{
		handleCameraMovement();
	}
	
	setCursor(Qt::BlankCursor);
}

void GLWidget::setupTimer()
{
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(0);
}

void GLWidget::keyPressEvent(QKeyEvent *e)
{
	if (e->key() == Qt::Key_M)
		use_special_controls = !use_special_controls;

	pressKey(e->key());
	QGLWidget::keyPressEvent(e);
}

void GLWidget::keyReleaseEvent(QKeyEvent *e)
{
	releaseKey(e->key());
	QGLWidget::keyPressEvent(e);
}

void GLWidget::mousePressEvent(QMouseEvent *e)
{
	QGLWidget::mousePressEvent(e);
	if (use_special_controls && e->button() == Qt::LeftButton)
		special_control_state = (special_control_state + 1) % 3;
	else
	{
		m_oldMousePos = e->pos();
		QCursor::setPos(mapToGlobal(QPoint(size().width() / 2, size().height() / 2)));
		m_mouseCapturing = true;
	}
	pressKey(e->button());
	setFocus();
}

void GLWidget::mouseReleaseEvent(QMouseEvent *e)
{
	QGLWidget::mouseReleaseEvent(e);

	if (e->button() == Qt::MouseButton::LeftButton && !use_special_controls)
	{
		QCursor::setPos(mapToGlobal(m_oldMousePos));
		m_mouseCapturing = false;
	}

	releaseKey(e->button());
}

void GLWidget::mouseMoveEvent(QMouseEvent *e)
{
	QGLWidget::mouseMoveEvent(e);
}

void GLWidget::handleCameraMovement()
{
	auto currMouse = mapFromGlobal(QCursor::pos());
	QPoint midPoint(size().width() / 2, size().height() / 2);
	auto diff = mapFromGlobal(QCursor::pos()) - midPoint;
	QCursor::setPos(mapToGlobal(midPoint));
	if (use_special_controls)
	{

		switch (special_control_state)
		{
		case 0:
			m_camera.backward(m_cameraSpeed * 0.002 * diff.y());
			m_camera.rotateRight(MOUSE_UNIT * 2 * diff.x());
			break;
		case 1:
			m_camera.right(m_cameraSpeed * 0.002 * diff.x());
			m_camera.down(m_cameraSpeed * 0.002 * diff.y());
			break;
		case 2:
			m_camera.rotateDown(MOUSE_UNIT * 2 * diff.y());
			m_camera.rotateRight(MOUSE_UNIT * 2 * diff.x());
			break;
		}
	}
	else
	{
		if (m_pressedKeys[Qt::MouseButton::LeftButton])
		{
			if (m_pressedKeys[Qt::MouseButton::RightButton])
			{
				setCursor(Qt::SizeAllCursor);
				m_camera.right(m_cameraSpeed * 0.002 * diff.x());
				m_camera.down(m_cameraSpeed * 0.002 * diff.y());
			}
			else
			{
				m_camera.rotateDown(MOUSE_UNIT * 2 * diff.y());
				m_camera.rotateRight(MOUSE_UNIT * 2 * diff.x());
			}
		}
		else if (m_pressedKeys[Qt::MouseButton::RightButton])
		{
			setCursor(Qt::SizeVerCursor);
			m_camera.backward(m_cameraSpeed * 0.002 * diff.y());
		}
	}
}

void GLWidget::setBGColor(const DirectGL::Texturing::Color3f &color)
{
	m_color = color;
}

void GLWidget::setReadBuffer(DirectGL::Buffers::Renderbuffer &buff)
{
	m_extBuff = &buff;
}

namespace
{
	std::string getExtension(const std::string &filename)
	{
		assert(filename.size());
		return filename.substr(filename.find_last_of("."));
	}
}

void GLWidget::loadModel(const std::string &model)
{
	assert(model.size());
	std::string test = getExtension(model);
	if (getExtension(model) == ".vgr")
	{
#ifdef VGR
		if (m_vgr_available)
		{
			m_model.clear();
			vgrAttributes initAttribs = vgrCreateAttributes(vgr_class_opendatabase);
			if (vgrOpenDatabase(&m_database, model.c_str(),
				vgr_db_open_existing | vgr_db_read_only | vgr_db_compatibility_mode_on, initAttribs) != vgr_result_ok)
				throw std::exception("Could not open database!");
			vgrSetCurrentDatabase(m_database);
			vgrDestroyAttributes(initAttribs);
			//vgrSetDatabaseMatrix(m_database, m_vgrFlip.data());
		}
#endif
	}
	else
	{

#ifdef VGR
		if (m_database != VGR_INVALID_DATABASE)
		{
			vgrCloseDatabase();
			m_database = NULL;
		}
#endif
		if (getExtension(model) == ".con")
		{
			Config conf(model);
			setSeparation(conf.getEntry<float>("separation"));
			setCameraSpeed(conf.getEntry<float>("cameraSpeed"));
			setScale(conf.getEntry<float>("scale"));
			setLodThresh(conf.getEntry<float>("lodThresh"));
			setNear(conf.getEntry<float>("near"));
			setFar(conf.getEntry<float>("far"));
			auto file = conf.getEntry<std::string>("targetFile");
			loadModel(file);
			m_camera.setProjective(90.f, m_near, m_far, static_cast<float>(size().width()) / size().height());
		}
		else
			m_model.fromFile(model);
	}
	m_camera.resetTransformation();
}

void GLWidget::pickModel(float x, float y) {
	//TODO ray cast
	picked_model = &m_model;
}