#!/bin/sh
# This script is used to build the sbgCom library on unix systems.
# To compile the library, you have to specify the byte ordering.
# Example: ./build.sh SBG_PLATFORM_LITTLE_ENDIAN

# Test that we have the endianness argument
if [ $# -ne 1 ]; then
    echo "You have to specify the platform endianness using either SBG_PLATFORM_BIG_ENDIAN or SBG_PLATFORM_LITTLE_ENDIAN"
    exit 1
fi

# Test the first argument and define the GCC options according to the selected endianness
if [ "$1" = "SBG_PLATFORM_BIG_ENDIAN" ]; then
    # The platform is in big endian
    gccOptions="-c -Wall -D SBG_PLATFORM_BIG_ENDIAN"
elif [ "$1" = "SBG_PLATFORM_LITTLE_ENDIAN" ]; then
    # The platform is in little endian
    gccOptions="-c -Wall -D SBG_PLATFORM_LITTLE_ENDIAN"
else
    echo "You have entered an invalid argument"
    exit 1
fi

# Create the intermediate directory		
mkdir obj

# Create all objects for binary logs directory
gcc $gccOptions ../../src/binaryLogs/binaryLogDebug.c -o obj/binaryLogDebug.o
gcc $gccOptions ../../src/binaryLogs/binaryLogDvl.c -o obj/binaryLogDvl.o
gcc $gccOptions ../../src/binaryLogs/binaryLogEkf.c -o obj/binaryLogEkf.o
gcc $gccOptions ../../src/binaryLogs/binaryLogEvent.c -o obj/binaryLogEvent.o
gcc $gccOptions ../../src/binaryLogs/binaryLogGps.c -o obj/binaryLogGps.o
gcc $gccOptions ../../src/binaryLogs/binaryLogImu.c -o obj/binaryLogImu.o
gcc $gccOptions ../../src/binaryLogs/binaryLogMag.c -o obj/binaryLogMag.o
gcc $gccOptions ../../src/binaryLogs/binaryLogOdometer.c -o obj/binaryLogOdometer.o
gcc $gccOptions ../../src/binaryLogs/binaryLogPressure.c -o obj/binaryLogPressure.o
gcc $gccOptions ../../src/binaryLogs/binaryLogs.c -o obj/binaryLogs.o
gcc $gccOptions ../../src/binaryLogs/binaryLogShipMotion.c -o obj/binaryLogShipMotion.o
gcc $gccOptions ../../src/binaryLogs/binaryLogStatus.c -o obj/binaryLogStatus.o
gcc $gccOptions ../../src/binaryLogs/binaryLogUsbl.c -o obj/binaryLogUsbl.o
gcc $gccOptions ../../src/binaryLogs/binaryLogUtc.c -o obj/binaryLogUtc.o

# Create all objects for commands directory
gcc $gccOptions ../../src/commands/commandsAdvanced.c -o obj/commandsAdvanced.o
gcc $gccOptions ../../src/commands/commandsCommon.c -o obj/commandsCommon.o
gcc $gccOptions ../../src/commands/commandsEvent.c -o obj/commandsEvent.o
gcc $gccOptions ../../src/commands/commandsGnss.c -o obj/commandsGnss.o
gcc $gccOptions ../../src/commands/commandsInfo.c -o obj/commandsInfo.o
gcc $gccOptions ../../src/commands/commandsInterface.c -o obj/commandsInterface.o
gcc $gccOptions ../../src/commands/commandsMag.c -o obj/commandsMag.o
gcc $gccOptions ../../src/commands/commandsOdo.c -o obj/commandsOdo.o
gcc $gccOptions ../../src/commands/commandsOutput.c -o obj/commandsOutput.o
gcc $gccOptions ../../src/commands/commandsSensor.c -o obj/commandsSensor.o
gcc $gccOptions ../../src/commands/commandsSettings.c -o obj/commandsSettings.o


# Create all objects for interfaces directory
gcc $gccOptions ../../src/interfaces/interfaceFile.c -o obj/interfaceFile.o
gcc $gccOptions ../../src/interfaces/interfaceUdp.c -o obj/interfaceUdp.o
gcc $gccOptions ../../src/interfaces/interfaceSerialUnix.c -o obj/interfaceSerialUnix.o

# Create all objects for misc directory
gcc $gccOptions ../../src/misc/sbgCrc.c -o obj/sbgCrc.o
gcc $gccOptions ../../src/misc/transfer.c -o obj/transfer.o

# Create all objects for protocol directory
gcc $gccOptions ../../src/protocol/protocol.c -o obj/protocol.o

# Create all objects for time directory
gcc $gccOptions ../../src/time/sbgTime.c -o obj/sbgTime.o

# Create all objets for the root directory
gcc $gccOptions ../../src/sbgECom.c -o obj/sbgECom.o

# Create the library
ar cr ../../libSbgECom.a obj/binaryLogDebug.o obj/binaryLogDvl.o obj/binaryLogEkf.o obj/binaryLogEvent.o obj/binaryLogGps.o obj/binaryLogImu.o obj/binaryLogMag.o obj/binaryLogOdometer.o obj/binaryLogPressure.o obj/binaryLogs.o obj/binaryLogShipMotion.o obj/binaryLogStatus.o obj/binaryLogUsbl.o obj/binaryLogUtc.o obj/commandsAdvanced.o obj/commandsCommon.o obj/commandsEvent.o obj/commandsGnss.o obj/commandsInfo.o obj/commandsInterface.o obj/commandsMag.o obj/commandsOdo.o obj/commandsOutput.o obj/commandsSensor.o obj/commandsSettings.o obj/interfaceFile.o obj/interfaceUdp.o obj/interfaceSerialUnix.o obj/sbgCrc.o obj/transfer.o obj/protocol.o obj/sbgTime.o obj/sbgECom.o
