# Ros-2-Environment
This repository contains a ros humble environment for better simulating the most recent updates about the implemented Control and Localization algorithms 

First of all, if you are using noVnc instead that a local gui, run the following bash command:

```
    cd ~/start_env
    bash ./gui_initialize.sh
```

Then if the noVnc container is running properly, run the ros humble docker container as follows:

```
    cd ~/start_env
    ./start.sh
```


Then, after the ros2 container is running, run in a separate terminal the launch file which spwans the robot inside gazebo environment.

```
    ros2 launch minimal_robot_gazebo spawn_robot.launch.py
```

After, in a new terminal (tmux) session you can verify which topics and types are published by gazebo:

```
    ros2 topic list -t
```
