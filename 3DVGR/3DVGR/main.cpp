#include "window3d.h"
#include <QtWidgets/QApplication>

#ifndef NDEBUG
#define _CRTDBG_MAP_ALLOC #include <stdlib.h> #include <crtdbg.h>
#endif

int main(int argc, char *argv[])
{
#ifndef NDEBUG
	_CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG);
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(2265);
#endif

	QApplication a(argc, argv);
	Window3D w;
	w.show();
	return a.exec();
}
