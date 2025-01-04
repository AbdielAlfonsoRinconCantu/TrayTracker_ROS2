#!/bin/bash

gnome-terminal --title="ROS2 computation graph" -- bash -c "
sleep 1;
cd;

echo 'rqt_graph';
rqt_graph;

bash;
"


gnome-terminal --title="Camera video publishing" -- bash -c "
sleep 2;
cd;

echo 'cd /home/auto/DALTILE/prod/ROS2_Sarath18_OpenCV/ros2_ws';
cd /home/auto/DALTILE/prod/ROS2_Sarath18_OpenCV/ros2_ws;

echo 'source install/setup.bash';
source install/setup.bash;

echo 'ros2 run ros2_opencv image_node';
ros2 run ros2_opencv image_node;

bash;
"


gnome-terminal --title="Object recognition" -- bash -c "
sleep 10;
cd;

echo 'cd /home/auto/DALTILE/prod/ROS2_Ab_YOLO/ros2_ws';
cd /home/auto/DALTILE/prod/ROS2_Ab_YOLO/ros2_ws;

echo 'source install/setup.bash';
source install/setup.bash;

echo 'source /home/auto/DALTILE/prod/ROS2_Ab_YOLO/yolo_test_venv_3/bin/activate';
source /home/auto/DALTILE/prod/ROS2_Ab_YOLO/yolo_test_venv_3/bin/activate;

echo 'ros2 run ros2_tracking tracker_node';
ros2 run ros2_tracking tracker_node;

bash;
"

gnome-terminal --title="Penny constructor" -- bash -c "
sleep 12;
cd;

echo 'cd /home/auto/DALTILE/prod/ROS2_Ab_C++/ros2_ws';
cd /home/auto/DALTILE/prod/ROS2_Ab_C++/ros2_ws;

echo 'source install/setup.bash';
source install/setup.bash;

echo 'ros2 run ros2_penny_constructor penny_tray_constructor';
ros2 run ros2_penny_constructor penny_tray_constructor

"
