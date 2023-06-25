#!/bin/bash
WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
echo "Workspace is $WS"
cd $WS
if [ "$1" = "Debug" ] || [ "$1" = "Release" ]
then
    mkdir -p build/$1 && cd build/$1 && cmake ../.. -DCMAKE_BUILD_TYPE=$1 -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON && cmake --build .
else
    echo "build.sh Error: Build options are 'Debug' or 'Release'"
    exit 1
fi