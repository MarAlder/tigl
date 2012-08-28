######################################################################
# Automatically generated by qmake (2.01a) Thu 26. Feb 23:08:35 2011
######################################################################

TEMPLATE = app
TARGET = TIGLViewer
DEPENDPATH += . 
INCLUDEPATH += . $(CASROOT)/inc ../Src ../../tixi/src /opt/opencascade/inc src
LIBS += -L../../tixi/lib -L../Src

# Input
CONFIG  += qt dll debug_and_release build_all 
CONFIG  += opengl 
HEADERS += src/TIGLViewerApplication.h\
           src/TIGLViewerContext.h \
           src/TIGLViewerWidget.h \
           src/TIGLViewerInputoutput.h \
           src/TIGLViewerDocument.h \
           src/TIGLViewerWindow.h \
           src/TIGLViewer.h \
           src/TIGLViewerInternal.h \
           src/CommandLineParameters.h \
           src/ISession_Point.h \
           src/ISession_Text.h \
           src/TIGLDebugStream.h 
           
SOURCES += 	src/main.cpp \
	   		src/TIGLViewerApplication.cpp \
	   		src/TIGLViewerContext.cpp \
           	src/TIGLViewerWidget.cpp \
           	src/TIGLViewerInputoutput.cpp \
           	src/TIGLViewerDocument.cpp \
           	src/TIGLViewerWindow.cpp \
           	src/CommandLineParameters.cpp \
            src/ISession_Point.cpp \
            src/ISession_Text.cpp
                      
DEFINES += CSFDB QOCC_STATIC

CONFIG(debug, debug|release) {
	message ("Building Debug Version") 
	DEFINES += _DEBUG
}

CONFIG(release, debug|release) {
	message ("Building Release Version") 
}

win32 {
	message ("Building for Win32") 
	DEFINES += WNT
	CONFIG += embed_manifest_dll windows
	
	PRECOMPILED_HEADER = src/TIGLViewerinternal.h
	include( TIGLViewerWin32.pri )
}

unix {
	message ("Building for Linux") 
	DEFINES += LIN LININTEL HAVE_CONFIG_H HAVE_IOSTREAM HAVE_FSTREAM HAVE_LIMITS_H
	include( TIGLViewerLinux.pri )
	
	# These flags are not necessary for compiling TIGLViewer, however they are if you compile OpenCascade
	# code with gcc 4.x
	# QMAKE_CXXFLAGS += -fpermissive -ffriend-injection

	HARDWARE_PLATFORM = $$system(uname -m)
	contains( HARDWARE_PLATFORM, x86_64 ) {
        	# 64-bit Linux
		message ("Adding Linux 64 bits compile flags and definitions") 
		DEFINES += _OCC64
		QMAKE_CXXFLAGS += -m64

	} else {
        	# 32-bit Linux
	}
}

FORMS += \
    src/TIGLViewerWindow.ui
