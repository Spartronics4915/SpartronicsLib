#!/usr/bin/env bash

YEAR=2019
REALSENSE_VERSION_SHORT="2.29"
REALSENSE_VERSION=$REALSENSE_VERSION_SHORT".0"
LIB_PATH=$(readlink -f ~/.gradle/toolchains/frc/$YEAR/roborio/arm-frc$YEAR-linux-gnueabi/lib)
RIO_LIB_PATH="/usr/local/frc/third-party/lib/"

scp "$LIB_PATH/librealsense2.so.$REALSENSE_VERSION" admin@10.49.15.2:"$RIO_LIB_PATH"
scp "$LIB_PATH/libusb-1.0.so.1.1.0" admin@10.49.15.2:/lib/libusb-1.0.so.1
scp "./src/native/build_cross/libspartronicsnative.so" lvuser@10.49.15.2:/home/lvuser/

ssh admin@10.49.15.2 ln -s "$RIO_LIB_PATH""librealsense2.so.$REALSENSE_VERSION" "$RIO_LIB_PATH""librealsense2.so.$REALSENSE_VERSION_SHORT"
