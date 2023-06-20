WS=$(cd $(dirname $0) && pwd)
echo "Working from $WS"
cd $WS

# make sure to chmod +x the python file and this script
FILE="P2D/scripts/show_results.py"
chmod +x $FILE
./$FILE "$@" > show_results.log
echo "Output redirected to 'show_results.log'"