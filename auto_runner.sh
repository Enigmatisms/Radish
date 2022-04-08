
methods=('bosch')
for file in `find /home/stn/slam/trajectories/ -maxdepth 1 -type d -name "hfps*"`; do
    echo "Processing ${file}/"
    output_f="${file}/eval_output/"
    if [ ! -d ${output_f} ]; then
        mkdir -p ${output_f}
    fi
    for method in ${methods[@]}; do
        python3 ./traj_eval.py "${file}/" ${method} &> "${output_f}${method}_odom_result.txt"
    done
done