#!/bin/bash
WS=$(cd $(dirname $0) && pwd)
echo "Working from $WS"
cd $WS

FILE="build/Release/run/run_exe"

# ================ Use the application directly by passing arguments to the application ==========================
# ./$FILE "$@" 

# ================ Store all the different results we want to compare ======================
./$FILE --algs VG2B R2E R2 --ids -1 --num_expts 1 --result_dir results --map_dir data --scen_dir data --names \
    dao/arena \
    # random/random512-10-1_scale2 \
    # dao/hrt201n_scale2 \
    # da2/ht_mansion2b_scale2 \
    # da2/ht_0_hightown_scale2 \
    # room/32room_000_scale2 \
    # room/16room_000_scale2 \
    # bg512/AR0709SR_scale2 \
    # bg512/AR0504SR_scale2 \
    # bg512/AR0014SR_scale2 \
    # bg512/AR0304SR_scale2 \
    # bg512/AR0702SR_scale2 \
    # bg512/AR0205SR_scale2 \
    # bg512/AR0602SR_scale2 \
    # bg512/AR0603SR_scale2 \
    # street/Denver_2_1024_scale2 \
    # street/NewYork_0_1024_scale2 \
    # street/Shanghai_2_1024_scale2 \
    # street/Shanghai_0_1024_scale2 \
    # street/Sydney_1_1024_scale2
    # dao/arena \
    # dao/hrt201n \
    # sc1/Aftershock \
    # sc1/Aurora \
    # sc1/ArcticStation \
    # da2/ht_mansion2b \
    # da2/ht_0_hightown \
    # da2/lt_0_lowtown_a3_n_c \
    # room/32room_000 \
    # room/16room_000 \
    # bg512/AR0709SR \
    # bg512/AR0504SR \
    # bg512/AR0014SR \
    # bg512/AR0304SR \
    # bg512/AR0702SR \
    # bg512/AR0205SR \
    # bg512/AR0602SR \
    # bg512/AR0603SR \
    # street/Denver_2_1024 \
    # street/NewYork_0_1024 \
    # street/Shanghai_2_1024 \
    # street/Shanghai_0_1024 \
    # street/Sydney_1_1024    