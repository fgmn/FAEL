#!/bin/bash
# run_fael_sim_background.sh: 后台运行多个 roslaunch，并将各自输出重定向到日志文件

# 1) 定义参数
world_name="/home/zhengkr/fael_ws/src/FAEL/fael_planner/exploration_manager/worlds/forest.world"
x=40
y=40
z=1.5
max_time=800
WAIT_TIME=900  # 等待时长(秒)

# 2) 加载工作空间
source devel/setup.bash

# 3) 启动各 roslaunch，并将输出分别重定向到不同文件
#    注意要把 stdout 和 stderr 都重定向，常用写法是  > file 2>&1

echo "启动 sim_env..."
roslaunch exploration_manager sim_env.launch \
  world_name:=$world_name x:=$x y:=$y z:=$z max_time:=$max_time \
  > sim_env.log 2>&1 &
PID_SIM=$!
sleep 2

echo "启动 robot_move..."
roslaunch exploration_manager robot_move.launch \
  > robot_move.log 2>&1 &
PID_MOVE=$!
sleep 2

echo "启动 explorer..."
roslaunch exploration_manager explorer.launch \
  > explorer.log 2>&1 &
PID_EXP=$!
sleep 2

echo "已后台启动 sim_env, robot_move, explorer."
echo "日志文件分别为 sim_env.log, robot_move.log, explorer.log"
echo "等待 $WAIT_TIME 秒后结束它们..."

sleep $WAIT_TIME

# 4) 结束三个后台进程
kill $PID_SIM $PID_MOVE $PID_EXP 2>/dev/null
sleep 2
kill -9 $PID_SIM $PID_MOVE $PID_EXP 2>/dev/null

echo "已结束三个后台进程。脚本退出。"
