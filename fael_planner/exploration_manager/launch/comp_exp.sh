#!/bin/bash
# launch_fael_sim.sh: 启动 FAEL 仿真并在指定时间后关闭新开的 gnome-terminal

# PASSWORD="zhengkairao"  # 用于 sudo 的密码

# 1) 定义参数
world_name="/home/zhengkr/fael_ws/src/FAEL/fael_planner/exploration_manager/worlds/comp_exp/forest_80.world"
dataset_path="/home/zhengkr/eval_map/forest"
log_path="$dataset_path/FAEL"

if [ -d "$log_path" ]; then
    rm -rf "$log_path"
fi

mkdir -p "$log_path"
free_points_files=$(find "$dataset_path" -type f -name "*free_points*.txt")
max_time=1000

x_coords=()
y_coords=()

while IFS= read -r file; do
    if [ -f "$file" ]; then
        mapfile -t lines < <(sed -n '2,16p' "$file")
        for line in "${lines[@]}"; do
            x=$(echo "$line" | awk '{print $1}')
            y=$(echo "$line" | awk '{print $2}')
            x_coords+=("$x")
            y_coords+=("$y")
        done
    fi
done <<< "$free_points_files"

# 打印关键信息
echo "World Name: $world_name"
echo "Max Time: $max_time"
echo "Starting Points:"
for ((i=0; i<${#x_coords[@]}; i++)); do
    echo "($i): x=${x_coords[$i]}, y=${y_coords[$i]}"
done

for ((i=0; i<${#x_coords[@]}; i++)); do
    x=${x_coords[$i]}
    y=${y_coords[$i]}
    echo "Launching simulation for point ($i): x=$x, y=$y"

    # 记录启动脚本前的所有 gnome-terminal 进程ID
    pid0=$(ps aux | grep "[g]nome-terminal" | awk '{print $2}')

    # 2) 启动第一个终端
    gnome-terminal --disable-factory --title="sim_env" -- bash -l -c "
    source devel/setup.bash &&
    roslaunch exploration_manager sim_env.launch \
        world_name:=$world_name x:=$x y:=$y use_gui:=false headless:=true;
    exec bash" &
    sleep 5

    # 3) 启动第二个终端
    gnome-terminal --disable-factory --title="robot_move" -- bash -l -c "
    source devel/setup.bash &&
    roslaunch exploration_manager robot_move.launch;
    exec bash" &
    sleep 5

    # 4) 启动第三个终端
    gnome-terminal --disable-factory --title="explorer" -- bash -l -c "
    source devel/setup.bash &&
    roslaunch exploration_manager explorer.launch \
        use_planner_rviz:=false max_time:=$max_time;
    exec bash" &

    sleep $((max_time + 10))

    # 5) 查找脚本期间新增的 gnome-terminal 并结束
    pid1=$(ps aux | grep "[g]nome-terminal" | awk '{print $2}')
    pid_to_kill=$(comm -13 <(echo "$pid0" | sort) <(echo "$pid1" | sort))
    echo "killing pid: $pid_to_kill"

    # pid1=$(ps aux | grep "[g]nome-terminal" | awk '{print $2}')
    # pid2=$(ps aux | grep "[g]nome-terminal" | grep "exploration_manager" | awk '{print $2}')
    # pidx=$(comm -13 <(echo "$pid0" | sort) <(echo "$pid1" | sort))
    # pidy=$(comm -13 <(echo "$pid0" | sort) <(echo "$pid2" | sort))

    # echo "pidx: $pidx"
    # echo "pidy: $pidy"

    # [ -n "$pidy" ] && kill -2 $pidy 2>/dev/null && sleep 3
    # [ -n "$pidx" ] && kill -2 $pidx 2>/dev/null && sleep 3

    [ -n "$pid_to_kill" ] && kill -2 $pid_to_kill 2>/dev/null && sleep 3
    [ -n "$pid_to_kill" ] && kill -TERM $pid_to_kill 2>/dev/null && sleep 3
    [ -n "$pid_to_kill" ] && kill -KILL $pid_to_kill 2>/dev/null

    sleep 33

    cp /home/zhengkr/fael_ws/src/FAEL/files/exploration_data/comp_data.txt "$log_path/log_${x}_${y}.txt"
    
done

