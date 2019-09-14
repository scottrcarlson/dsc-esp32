#!/bin/bash

# usage: ./build_upload.sh /dev/ttyUSB0

SKETCH_FILENAME=dscv4_arduino_sketch.ino

MAC_ARDUINO_EXECUTABLE=/Applications/Arduino.app/Contents/MacOS/Arduino
LINUX_ARDUINO_EXECUTABLE=/usr/local/bin/arduino

echo "static const char GIT_REV[] = \"$(git describe --abbrev=4 --dirty --always --tags)\";" > build-time_source_code_git_version.h

if test -f ${MAC_ARDUINO_EXECUTABLE}; then
    # file exists
    ${MAC_ARDUINO_EXECUTABLE} --upload ${SKETCH_FILENAME} --port ${1}
else
    ${LINUX_ARDUINO_EXECUTABLE} --upload ${SKETCH_FILENAME} --port ${1}
fi

