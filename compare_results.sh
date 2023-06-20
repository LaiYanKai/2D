
# MAP_LIST=(
#     # "room 16room_000"     
#     # "room 32room_000"
#     # "da2 ht_mansion2b"
#     # "da2 ht_0_hightown"
#     # "street Denver_2_1024"
#     # "street NewYork_0_1024"
#     # "street Shanghai_2_1024"
#     # "street Shanghai_0_1024"
#     # "street Sydney_1_1024"
#     # "bg512 AR0709SR"
#     # "bg512 AR0504SR"
#     # "bg512 AR0014SR"
#     # "bg512 AR0304SR"
#     # "bg512 AR0702SR"
#     # "bg512 AR0205SR"
#     # "bg512 AR0602SR"
#     # "bg512 AR0603SR" 
#     # "dao arena"
#     # "dao hrt201n"
#     "da2 ca_caverns1"
#     )

# for MAP_PAIR in "${MAP_LIST[@]}"
# do
#     IFS=" " read -r -a pair <<< "${MAP_PAIR}"
#     MAP_DIR="${pair[0]}"    
#     MAP_NAME="${pair[1]}"
#     MAP_PATH=scenarios/$MAP_DIR/$MAP_NAME

#     echo "====== Generating Mesh for $MAP_PAIR ======================" 
#     ./converter -Lmai -i $MAP_PATH.map.scen -o $MAP_PATH.pfarc --conv=mesh --compute=st 
# done



WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
echo "Workspace is $WS"
cd $WS
FILE="P2D/scripts/compare_results.py"
chmod +x $FILE
# make sure to chmod +x this script

# ================ Use the compare_results directly ==========================
# ./$FILE "$@" 

# ================ Store all the different results we want to compare ======================
./$FILE --name results/da2/lt_0_lowtown_a3_n_c --algs VG2B VG2N