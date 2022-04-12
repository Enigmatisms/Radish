
path="/home/stn/Dataset/hfps_new/hfps/"
for file in `find ${path} -maxdepth 1 -type f -name "*.bag"`; do
    echo "Processing bag at: ${file}"
    python3 ./validator.py ${file}
done
if [ ! -d "${path}fix/" ]; then
    mkdir -p "${path}fix/"
fi
for file in `find ${path} -maxdepth 1 -type f -name "*_fix*"`; do
    mv ${file} "${path}fix/"
done