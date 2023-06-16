WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
echo "Workspace is $WS"
cd $WS

# make sure to chmod +x the python file and this script
python3 ./P2D/scripts/show_results.py "$@"