
for file in `find /home/stn/slam/trajectories/ -maxdepth 1 -type d -name "hfps*"`; do
    echo "Processing ${file}/"
    output_f="${file}/eval_output/"
    if [ ! -d ${output_f} ]; then
        mkdir -p ${output_f}
    fi
    all_nums=$(ls ${file}/carto* | wc -l)
    for((i=0;i<${all_nums};i++)); do
        python3 ./traj_eval.py --path="${file}/" --traj_num=${i} -f
    done
done