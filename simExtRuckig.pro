include(config.pri)

QT -= core
QT -= gui

TARGET = simExtRuckig
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += QT_COMPIL
CONFIG += shared plugin
INCLUDEPATH += "ruckig/include"
INCLUDEPATH += "../include"

*-msvc* {
    DEFINES += _USE_MATH_DEFINES
    DEFINES += NOMINMAX
	QMAKE_CXXFLAGS += -O2
	QMAKE_CXXFLAGS += -W3
    QMAKE_CXXFLAGS += -std:c++17
}
*-g++* {
    QMAKE_CXXFLAGS += -std=c++17
    QMAKE_CXXFLAGS += -O3
	QMAKE_CXXFLAGS += -Wall
	QMAKE_CXXFLAGS += -Wno-unused-parameter
	QMAKE_CXXFLAGS += -Wno-strict-aliasing
	QMAKE_CXXFLAGS += -Wno-empty-body
	QMAKE_CXXFLAGS += -Wno-write-strings

	QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
	QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
	QMAKE_CXXFLAGS += -Wno-narrowing

	QMAKE_CFLAGS += -O3
	QMAKE_CFLAGS += -Wall
	QMAKE_CFLAGS += -Wno-strict-aliasing
	QMAKE_CFLAGS += -Wno-unused-parameter
	QMAKE_CFLAGS += -Wno-unused-but-set-variable
	QMAKE_CFLAGS += -Wno-unused-local-typedefs
}

INCLUDEPATH += $$BOOST_INCLUDEPATH

win32 {
    DEFINES += WIN_SIM
}
win32-g++ {
    LIBS += -lshlwapi
}

macx {
    DEFINES += MAC_SIM
}

unix:!macx {
    DEFINES += LIN_SIM
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

HEADERS += \
    ../include/simLib.h \
    simExtRuckig.h \
    ruckig/include/ruckig/block.hpp \
    ruckig/include/ruckig/brake.hpp \
    ruckig/include/ruckig/input_parameter.hpp \
    ruckig/include/ruckig/output_parameter.hpp \
    ruckig/include/ruckig/position.hpp \
    ruckig/include/ruckig/profile.hpp \
    ruckig/include/ruckig/roots.hpp \
    ruckig/include/ruckig/ruckig.hpp \
    ruckig/include/ruckig/trajectory.hpp \
    ruckig/include/ruckig/velocity.hpp

SOURCES += \
    ../include/simLib.cpp \
    simExtRuckig.cpp \
    ruckig/src/brake.cpp \
    ruckig/src/position-step1.cpp \
    ruckig/src/position-step2.cpp \
    ruckig/src/velocity-step1.cpp \
    ruckig/src/velocity-step2.cpp
