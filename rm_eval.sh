
methods=('carto_' 'gmap_' 'c_traj_')
for file in `find /home/llk/slam/trajectories/ -maxdepth 1 -type d -name "hfps*"`; do
    echo "Removing ${file}/eval..."
    output_f="${file}/eval_output/"
    if [ ! -d ${output_f} ]; then
        mkdir -p ${output_f}
    fi
    for method in ${methods[@]}; do
        rm -f "${output_f}${method}_result.txt"
    done
    rm -rf ${file}/eval_figs/
done