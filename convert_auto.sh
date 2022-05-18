
input_path="/home/${USER}/Dataset/hfps_new/"
output_path="/home/${USER}/Dataset/hfps_new/odom2tf/"
echo "Finding bag files in ${input_path}..."
for file in $(find ${input_path} -maxdepth 1 -type f -name "*.bag"); do
    bag_name=${file##*"/"}
    output_bag="${output_path}${bag_name}"
    python ./slam_toolboxify.py ${file} ${output_bag}
    echo "${bag_name}: Process completed."
done