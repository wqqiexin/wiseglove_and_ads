QT += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11


RC_ICONS =xiaohui.ico
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Src/hzy.cpp \
    Src/modern_robotics.cpp \
    Src/vrepsim.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    Include/RemoteApi/extApi.h \
    Include/RemoteApi/extApiPlatform.h \
    Include/hzy.h \
    Include/modern_robotics.h \
    Include/vrepsim.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

TRANSLATIONS += \
    wiseglove_and_ads_zh_CN.ts
CONFIG += lrelease
CONFIG += embed_translations

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


INCLUDEPATH += $$PWD/Include

INCLUDEPATH += $$PWD/Include/RemoteApi
INCLUDEPATH += $$PWD/Lib
DEPENDPATH += $$PWD/Lib
INCLUDEPATH += C:\eigen-3.4.0
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/Dll/.. -lWISEGLOVE
LIBS += $$PWD/Lib/WISEGLOVE.lib
win32: LIBS += -L$$PWD/./ -lWISEGLOVE



DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=
DEFINES += DO_NOT_USE_SHARED_MEMORY


win32: LIBS += -L$$PWD/Lib/ -lremoteApiSharedLib-64

INCLUDEPATH += $$PWD/Lib
DEPENDPATH += $$PWD/Lib
