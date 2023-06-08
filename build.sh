if [ "$1" = "Debug" ] || [ "$1" = "Release" ]
then
    mkdir -p build/$1 && cd build/$1 # && cmake ../.. -DCMAKE_BUILD_TYPE=$1 && cmake --build .
else
    echo "build.sh Error: Build options are 'Debug' or 'Release'"
    exit 1
fi