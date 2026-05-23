QT += core serialport
QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

TARGET = SerialPortConnector_CLI

# keep CLI objects out of the GUI's in-place build so the two are independent
OBJECTS_DIR = obj_cli
MOC_DIR = moc_cli

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
    BF_ROOTLOADER.cpp \
    fourwayif.cpp \
    hexfile.cpp \
    cli_main.cpp

HEADERS += \
    BF_ROOTLOADER.h \
    fourwayif.h \
    hexfile.h \
    defaults.h
