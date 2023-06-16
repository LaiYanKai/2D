WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
echo "Workspace is $WS"
cd $WS

# make sure to chmod +x the python file and this script
./P2D/scripts/compare_results.py "$@" 

# > show_results.log
# echo "Output redirected to 'show_results.log'"