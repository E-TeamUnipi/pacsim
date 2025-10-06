# how to run


## build

- Creare un workspace ros2: 
    ```mkdir ros_ws```
- Clonare questa repo nel ws ```git clone ...```
- Fai il sourcing di ros ```source /opt/ros/...```
- build ```colcon build```
- Sourcing del progetto ```source install/setup.sh```


## run
- ```ros2 launch pacsim example.launch.py```
- monitorare stato macchina ```ros2 launch pacsim example.launch.py```
- lanciare bridge ros2-websocket per foxglove ```ros2 launch foxglove_bridge foxglove_bridge_launch.xml```
- apri foxglove, connetti al websocket server, poi alla voce topic rendi visibile `/pacsim/track/visualization/`