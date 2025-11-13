# how to run


## build

- Creare un workspace ros2: 
    ```mkdir ros_ws```
- Clonare questa repo nel ws ```git clone ...```
- Fai il sourcing di ros ```source /opt/ros/...```
- build ```colcon build```
- Sourcing del progetto ```source install/setup.sh```


## run
- ricordati di sourcare ros e il progetto ```source /opt/ros/jazzy/setup.zsh ; source install/setup.zsh```
- lanciare bridge ros2-websocket per foxglove ```ros2 launch foxglove_bridge foxglove_bridge_launch.xml```
- lancia pacsim ```ros2 launch pacsim example.launch.py```
- apri foxglove, connetti al websocket server, poi alla voce topic rendi visibile `/pacsim/track/visualization/`