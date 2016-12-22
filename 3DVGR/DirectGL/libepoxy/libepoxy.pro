#-------------------------------------------------
#
# Project created by QtCreator 2015-06-05T17:32:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = libepoxy
TEMPLATE = app
TEMPLATE = lib

HEADERS  += $$_PRO_FILE_PWD_/include/epoxy/gl.h \
			$$_PRO_FILE_PWD_/include/epoxy/gl_generated.h \
			$$_PRO_FILE_PWD_/src/dispatch_common.h
SOURCES  += $$_PRO_FILE_PWD_/src/dispatch_common.c \
			$$_PRO_FILE_PWD_/src/gl_generated_dispatch.c
			

win32 {
	HEADERS += $$_PRO_FILE_PWD_/include/epoxy/wgl.h \
				$$_PRO_FILE_PWD_/include/epoxy/wgl_generated.h
	SOURCES += $$_PRO_FILE_PWD_/src/wgl_generated_dispatch.c \
			   $$_PRO_FILE_PWD_/src/dispatch_wgl.c
}

!win32{
	HEADERS += $$_PRO_FILE_PWD_/include/epoxy/glx.h \
				$$_PRO_FILE_PWD_/include/epoxy/glx_generated.h
	SOURCES += $$_PRO_FILE_PWD_/src/glx_generated_dispatch.c \
			   $$_PRO_FILE_PWD_/src/dispatch_glx.c
}

contains(QT_ARCH, i386 ) {
    PLATFORM = win32
}else {
    PLATFORM = x64
}

CONFIG(debug, debug|release) {
    DESTDIR = $$_PRO_FILE_PWD_/../..//bin/$$PLATFORM/Debug
} else {
    DESTDIR = $$_PRO_FILE_PWD_/../../bin/$$PLATFORM/Release
}

INCLUDEPATH += "$$_PRO_FILE_PWD_/include"
DEPENDPATH += "$$_PRO_FILE_PWD_/include" 
