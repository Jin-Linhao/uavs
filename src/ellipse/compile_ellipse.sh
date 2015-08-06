#!/usr/bin/env bash
cd src/sbgECom/projects/unix
bash ./build.sh SBG_PLATFORM_LITTLE_ENDIAN
cd ../../../Examples/ellipseMinimal/projects/unix
bash ./build.sh
