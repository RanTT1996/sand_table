QT       += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings 
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    form.cpp \
    infomation_static.cpp \
    main.cpp \
    vehicleconsole.cpp \
    vehiclemotion.cpp \
    vehiclesensor.cpp

HEADERS += \
    chassiscontrol.h \
    form.h \
    infomation_static.h \
    vehicleconsole.h \
    vehiclemotion.h \
    vehiclesensor.h

FORMS += \
    form.ui \
    vehicleconsole.ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#添加RPlidar的静态库/头文件
unix:!macx: LIBS += -L$$PWD/../../library/librplidarA1/ -lrplidar_sdk
INCLUDEPATH += $$PWD/../../library/librplidarA1/include
DEPENDPATH += $$PWD/../../library/librplidarA1/include
unix:!macx: PRE_TARGETDEPS += $$PWD/../../library/librplidarA1/librplidar_sdk.a

#添加wiringpi的两个库
unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/ -lwiringPi
unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/ -lwiringPiDev
DEPENDPATH += $$PWD/../../../../usr/include

#添加realsense的库
unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lrealsense2
INCLUDEPATH += $$PWD/../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../usr/local/include

#添加opencv的库
unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lopencv_core
unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lopencv_highgui
unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lopencv_imgproc
unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lopencv_imgcodecs
INCLUDEPATH += $$PWD/../../../../usr/include/opencv4
DEPENDPATH += $$PWD/../../../../usr/include/opencv4
