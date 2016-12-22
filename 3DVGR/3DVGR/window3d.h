#ifndef WINDOW3D_H
#define WINDOW3D_H

#include <QtWidgets/QMainWindow>
#include "ui_window3d.h"

#include "GLWidget.h"

#include <string>

#include <DirectGL/DirectGL.h>

class Window3D : public QMainWindow
{
	Q_OBJECT

public:
	Window3D(QWidget *parent = 0);
	Window3D(Window3D *partner, QWidget *parent = 0);
	~Window3D();

protected:
	void keyReleaseEvent(QKeyEvent *e);

private:
	Ui::Window3DClass ui;
	Window3D *m_partner;
	GLWidget *m_glWidget;

	bool m_isoriginal;

	std::string m_currentFile;
	std::unique_ptr<DirectGL::Utility::Logging::VSDebug> m_log;

};

#endif // WINDOW3D_H
