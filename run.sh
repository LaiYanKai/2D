#!/bin/bash
WS=$(cd $(dirname $0) && pwd)
echo "Working from $WS"
cd $WS

FILE="build/Release/run/run_exe"

# ================ Use the compare_results directly ==========================
# ./$FILE "$@" 

# ================ Store all the different results we want to compare ======================
./$FILE --algs R2E R2 --ids -1 --num_expts 10 --names \
    dao/arena \
    dao/hrt201n \
    sc1/Aftershock \
    sc1/Aurora \
    sc1/ArcticStation \
    da2/ht_mansion2b \
    da2/ht_0_hightown \
    da2/lt_0_lowtown_a3_n_c \
    room/32room_000 \
    room/16room_000 \
    bg512/AR0709SR \
    bg512/AR0504SR \
    bg512/AR0014SR \
    bg512/AR0304SR \
    bg512/AR0702SR \
    bg512/AR0205SR \
    bg512/AR0602SR \
    bg512/AR0603SR \
    street/Denver_2_1024 \
    street/NewYork_0_1024 \
    street/Shanghai_2_1024 \
    street/Shanghai_0_1024 \
    street/Sydney_1_1024    

# if [ "$1" = "Debug" ] || [ "$1" = "Release" ]
# then
#     if [ "$2" = "Debug" ] || [ "$2" = "Release" ]
#     then
#         ./build/$1/$2/$2_exe
#     else
#         echo "run.sh Error: Algorithm must be 'VG2', 'TS2'"
#     fi
# else
#     echo "run.sh Error: Build options are 'Debug' or 'Release'"
#     exit 1
# fi