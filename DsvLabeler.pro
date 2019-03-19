TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG += qt

QT       += core gui opengl

INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/pcl-1.9 /usr/include/eigen3/ /usr/include/vtk-5.10
LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_viz
LIBS += /usr/lib/libpcl*.so /usr/lib/libvtk*.so /usr/lib/x86_64-linux-gnu/libboost*.so /usr/lib/x86_64-linux-gnu/libpcl_v*.so.1.7.2


SOURCES += main.cpp \
    CalcPlane.cpp \
    Calculation.cpp \
    ContourSegger.cpp \
    DataAssociation.cpp \
    LineSegger.cpp

SUBDIRS += \
    DsvLabeler.pro

HEADERS += \
    ContourSegger.h \
    DsvLabelMv.h \
    LineSegger.h

QMAKE_CXXFLAGS += -std=c++14
