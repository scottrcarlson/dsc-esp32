#!/bin/bash

# usage: ./build_upload.sh /dev/ttyUSB0

SKETCH_FILENAME=dscv4_arduino_sketch.ino

MAC_ARDUINO_EXECUTABLE=/Applications/Arduino.app/Contents/MacOS/Arduino
LINUX_ARDUINO_EXECUTABLE=/usr/local/bin/arduino

if test -f ${MAC_ARDUINO_EXECUTABLE}; then
    # file exists
    ${MAC_ARDUINO_EXECUTABLE} --upload ${SKETCH_FILENAME} --port ${1}
else
    ${LINUX_ARDUINO_EXECUTABLE} --upload ${SKETCH_FILENAME} --port ${1}
fi

