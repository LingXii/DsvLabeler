TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG += qt

QT       += core gui opengl

INCLUDEPATH += /usr/local/include \
    /usr/include/pcl-1.9 \
    /usr/include/eigen3 \
    /usr/include/vtk-6.2

LIBS += /usr/lib/libpcl* \
    /usr/lib/libvtk* \
    /usr/local/lib/libboost* \
    /usr/lib/x86_64-linux-gnu/libflann* \
    /usr/local/lib/libopencv*

SOURCES += main.cpp \
    CalcPlane.cpp \
    Calculation.cpp \
    ContourSegger.cpp \
    DataAssociation.cpp \
    LineSegger.cpp \
    pclmethod.cpp

SUBDIRS += \
    DsvLabeler.pro

HEADERS += \
    ContourSegger.h \
    DsvLabelMv.h \
    LineSegger.h \
    pclmethod.h

QMAKE_CXXFLAGS += -std=c++14
