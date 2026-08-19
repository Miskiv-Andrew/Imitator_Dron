QT       += core gui serialport widgets  printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS += -Wa,-mbig-obj

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    mapcore.cpp \
    Viewermap.cpp \
    pointdata.cpp \
    qcustomplot.cpp \
    surveyarea.cpp

HEADERS += \
    mainwindow.h \
    mapcore.h \
    Viewermap.h \
    pointdata.h \
    qcustomplot.h \
    surveyarea.h

FORMS += \
    mainwindow.ui

# LIBS += -lqcustomplot

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
