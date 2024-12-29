#!/bin/bash

run_in_terminal() {
    gnome-terminal -- bash -c "$1"
}

launch_command="cd ~/auro_ws; colcon build; source ~/auro_ws/install/setup.bash; ros2 launch assessment assessment_launch.py num_robots:=3; bash"

run_command1="echo controller0; sleep 20; source ~/auro_ws/install/setup.bash; ros2 run assessment controller0; bash"
run_command6="echo controller1; sleep 20; source ~/auro_ws/install/setup.bash; ros2 run assessment controller1; bash"
run_command7="echo controller2; sleep 20; source ~/auro_ws/install/setup.bash; ros2 run assessment controller2; bash"
run_command2="echo item_manager; sleep 20; source ~/auro_ws/install/setup.bash; ros2 run assessment item_manager --random_seed 42; exit"
run_command3="echo item_sensor; sleep 20; source ~/auro_ws/install/setup.bash; ros2 run assessment item_sensor; exit"
run_command4="echo zone_sensor; sleep 20; source ~/auro_ws/install/setup.bash; ros2 run assessment zone_sensor; exit"
run_command5="echo robot_sensor; sleep 20; source ~/auro_ws/install/setup.bash; ros2 run assessment robot_sensor; exit"

run_in_terminal "$launch_command"
run_in_terminal "$run_command2"
run_in_terminal "$run_command3"
run_in_terminal "$run_command4"
run_in_terminal "$run_command5"
run_in_terminal "$run_command1"
run_in_terminal "$run_command6"
run_in_terminal "$run_command7"

echo "Done"
