TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += $$PWD\BLE
INCLUDEPATH += $$PWD\BLEGrill_nRF8001

include(deployment.pri)
qtcAddDeployment()

