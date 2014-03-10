#-------------------------------------------------
#
# Project created by QtCreator 2013-12-20T17:10:13
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = cwork_complex_qt
CONFIG   += console
CONFIG   -= app_bundle
CONFIG += c++11

TEMPLATE = app


SOURCES += main.cpp
SOURCES += headers\Constellation.h
SOURCES += headers\DrawGraphics.h
SOURCES += headers\IntegrationMethod.h
SOURCES += headers\KalmanFilter.h
SOURCES += headers\Model.h
SOURCES += headers\RightPart.h
SOURCES += headers\SystemSimulateNavigationSignals.h
SOURCES += headers\SystemSimulation.h
SOURCES += headers\vector_matrix.h
SOURCES += headers\WhiteNoiseGenerator.h


INCLUDEPATH += C:\Users\artem\Documents\GitHub\cwork_complex_qt\cwork_complex_qt\boost
INCLUDEPATH += C:\Users\artem\Documents\GitHub\cwork_complex_qt\cwork_complex_qt

