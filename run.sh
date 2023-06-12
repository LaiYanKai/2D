WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
echo "Workspace is $WS"
cd $WS
source build_type.sh

if [ "$1" = "Debug" ] || [ "$1" = "Release" ]
then
    if [ "$2" = "Debug" ] || [ "$2" = "Release" ]
    then
        ./build/$1/$2/$2_exe
    else
        echo "run.sh Error: Algorithm must be 'VG2', 'TS2'"
    fi
else
    echo "run.sh Error: Build options are 'Debug' or 'Release'"
    exit 1
fi