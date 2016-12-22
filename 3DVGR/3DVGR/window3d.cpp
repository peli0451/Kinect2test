#include "window3d.h"
#include "Config.h"

#include <QKeyEvent>
#include <QFileDialog>

Window3D::Window3D(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	
	m_log.reset(new DirectGL::Utility::Logging::VSDebug());

	m_partner = new Window3D(this);
	m_partner->show();

	QGLFormat format;
	format.setProfile(QGLFormat::CoreProfile);
	format.setDepthBufferSize(24);
	format.setRgba(true);
	format.setVersion(3, 3);
	format.setDoubleBuffer(true);
	m_glWidget = new GLWidget(format, this, m_partner->m_glWidget);
	setCentralWidget(m_glWidget);

	m_isoriginal = true;
}

Window3D::Window3D(Window3D *partner, QWidget *parent)
{
	ui.setupUi(this);
	assert(partner);

	QGLFormat format;
	format.setProfile(QGLFormat::CoreProfile);
	format.setDepthBufferSize(24);
	format.setRgba(true);
	format.setVersion(3, 3);
	format.setDoubleBuffer(true);
	m_glWidget = new GLWidget(format, this);
	setCentralWidget(m_glWidget);
	m_glWidget->setBGColor(DirectGL::Texturing::Color3f(1.f, 0.f, 0.f));
	m_partner = partner;
	m_isoriginal = false;
}

Window3D::~Window3D()
{
	if(m_isoriginal)
		delete m_partner;
}

void Window3D::keyReleaseEvent(QKeyEvent *e)
{
	if (m_isoriginal)
	{
		switch (e->key())
		{
		case Qt::Key_Escape:
			this->close();
			m_partner->close();
			break;
		case Qt::Key_F:
			switch (this->windowState())
			{
			case Qt::WindowState::WindowFullScreen:
				this->setWindowState(Qt::WindowState::WindowNoState);
				m_partner->setWindowState(Qt::WindowState::WindowNoState);
				break;
			default:
				this->setWindowState(Qt::WindowState::WindowFullScreen);
				m_partner->setWindowState(Qt::WindowState::WindowFullScreen);
				break;
			}
			break;
		case Qt::Key_O:
		{
						  m_glWidget->setUpdatesEnabled(false);
						  QString filename = QFileDialog::getOpenFileName(this, "Add model...", "", "Model Files (*.obj *.vgr *.con);;OBJ models (*.obj);;VGR databases (*.vgr);;Config Model files (*.con)");
						  if (filename.size())
						  {
							  m_currentFile = filename.toStdString();
							  m_glWidget->loadModel(m_currentFile);
						  }
						  m_glWidget->setUpdatesEnabled(true);
						  break;
		}
		case Qt::Key_Plus:
			m_glWidget->increaseScale();
			break;
		case Qt::Key_Minus:
			m_glWidget->decreaseScale();
			break;
		case Qt::Key_PageUp:
			m_glWidget->increaseScale(0.0001);
			break;
		case Qt::Key_PageDown:
			m_glWidget->decreaseScale(0.0001);
			break;
		case Qt::Key_Left:
			m_glWidget->increaseSeparation();
			break;
		case Qt::Key_Right:
			m_glWidget->decreaseSeparation();
			break;
		case Qt::Key_Up:
			m_glWidget->increaseCameraSpeed();
			break;
		case Qt::Key_Down:
			m_glWidget->decreaseCameraSpeed();
			break;
		case Qt::Key_1:
			m_glWidget->setLodThresh(0.1f);
			break;
		case Qt::Key_2:
			m_glWidget->setLodThresh(0.2f);
			break;
		case Qt::Key_3:
			m_glWidget->setLodThresh(0.3f);
			break;
		case Qt::Key_4:
			m_glWidget->setLodThresh(0.4f);
			break;
		case Qt::Key_5:
			m_glWidget->setLodThresh(0.5f);
			break;
		case Qt::Key_6:
			m_glWidget->setLodThresh(0.6f);
			break;
		case Qt::Key_7:
			m_glWidget->setLodThresh(0.7f);
			break;
		case Qt::Key_8:
			m_glWidget->setLodThresh(0.8f);
			break;
		case Qt::Key_9:
			m_glWidget->setLodThresh(0.9f);
			break;
		case Qt::Key_0:
			m_glWidget->setLodThresh(1.f);
			break;
		case Qt::Key_N:
			{
				m_glWidget->setUpdatesEnabled(false);
				auto name = QFileDialog::getSaveFileName(this, "Save config...", "", "Model Config Files (*.con)");
				if (name.size())
				{
					Config conf;
					conf.setEntry("targetFile", m_currentFile);
					conf.setEntry("scale", m_glWidget->getScale());
					conf.setEntry("separation", m_glWidget->getSeparation());
					conf.setEntry("cameraSpeed", m_glWidget->getCameraSpeed());
					conf.setEntry("lodThresh", m_glWidget->getLodThresh());
					conf.setEntry("near", m_glWidget->getNear());
					conf.setEntry("far", m_glWidget->getFar());
					conf.toFile(name.toStdString());
				}
				m_glWidget->setUpdatesEnabled(true);
			}
		}
		QMainWindow::keyReleaseEvent(e);
	}
	else
		m_partner->keyReleaseEvent(e);
}