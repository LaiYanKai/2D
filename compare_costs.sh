WS=$(cd $(dirname $0) && pwd)
echo "Working from $WS"
cd $WS
FILE="P2D/scripts/compare_costs.py"
chmod +x $FILE
# make sure to chmod +x this script

# ================ Use the compare_results directly ==========================
# ./$FILE "$@" 

# ================ Store all the different results we want to compare ======================
./$FILE --algs VG2B ANYA2B --name \
    results/dao/arena \
    results/dao/hrt201n \
    results/sc1/Aftershock \
    results/sc1/Aurora \
    results/sc1/ArcticStation \
    results/da2/ht_mansion2b \
    results/da2/ht_0_hightown \
    results/da2/lt_0_lowtown_a3_n_c \
    results/room/32room_000 \
    results/room/16room_000 \
    results/bg512/AR0709SR \
    results/bg512/AR0504SR \
    results/bg512/AR0014SR \
    results/bg512/AR0304SR \
    results/bg512/AR0702SR \
    results/bg512/AR0205SR \
    results/bg512/AR0602SR \
    results/bg512/AR0603SR \
    results/street/Denver_2_1024 \
    results/street/NewYork_0_1024 \
    results/street/Shanghai_2_1024 \
    results/street/Shanghai_0_1024 \
    results/street/Sydney_1_1024 \
    --print compare_results.log
