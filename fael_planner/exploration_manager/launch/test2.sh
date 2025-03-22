#!/bin/bash
# launch_fael_sim.sh: 启动 FAEL 仿真并在指定时间后关闭新开的 gnome-terminal

# PASSWORD="zhengkairao"  # 用于 sudo 的密码

# 1) 定义参数
world_name="/home/zhengkr/fael_ws/src/FAEL/fael_planner/exploration_manager/worlds/comp_exp/forest_80.world"
x=11.100000 
y=58.000000
z=1.5
max_time=6666
WAIT_TIME=6666  # 等待时长（秒），这里示例为15分钟

# 记录启动脚本前的所有 gnome-terminal 进程ID
PIDS_BEFORE=$(ps aux | grep "[g]nome-terminal" | awk '{print $2}')

# 2) 启动第一个终端
gnome-terminal --disable-factory -- bash -l -c "
  source devel/setup.bash &&
  roslaunch exploration_manager sim_env.launch \
    world_name:=$world_name x:=$x y:=$y max_time:=$max_time use_gui:=true;
  exec bash" &
sleep 5

# 3) 启动第二个终端
gnome-terminal --disable-factory -- bash -l -c "
  source devel/setup.bash &&
  roslaunch exploration_manager robot_move.launch;
  exec bash" &
sleep 5

# 4) 启动第三个终端
gnome-terminal --disable-factory -- bash -l -c "
  source devel/setup.bash &&
  roslaunch exploration_manager explorer.launch;
  exec bash" &

echo "已启动三扇终端，等待 $WAIT_TIME 秒后关闭..."

sleep $WAIT_TIME

# 5) 查找脚本期间新增的 gnome-terminal 并结束
PIDS_AFTER=$(ps aux | grep "[g]nome-terminal" | awk '{print $2}')
PIDS_TO_KILL=$(comm -13 <(echo "$PIDS_BEFORE" | sort) <(echo "$PIDS_AFTER" | sort))

[ -n "$PIDS_TO_KILL" ] && kill -TERM $PIDS_TO_KILL 2>/dev/null && sleep 2
[ -n "$PIDS_TO_KILL" ] && kill -KILL $PIDS_TO_KILL 2>/dev/null

# 等待所有节点结束
sleep 30

# rosnode kill -a
# echo $PASSWORD | sudo killall -9 roscore rosmaster
echo "结束新开终端，脚本退出。"
