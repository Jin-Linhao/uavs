#!/bin/sh
# This script is used to build the ellipseOnboardMagCalib example on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ellipseOnboardMagCalib.c  -I../../../../sbgECom/src/ ../../../../sbgECom/libSbgECom.a -o ../../ellipseOnboardMagCalib

