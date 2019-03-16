TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG += qt

QT       += core gui opengl

INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_viz


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
