#!/bin/sh
# This script is used to build the ellipseMinimal example on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ellipseMinimal.c  -I../../../../sbgECom/src/ ../../../../sbgECom/libSbgECom.a -o ../../ellipseMinimal

