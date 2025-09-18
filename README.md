**INSTRUCTION FOR RUNNING THE SIMULATION**

Here are reported all the instructions for running the simulation on gazebo with **falco_drone** and a brief guide regarding how the control system is implemented.

First, build the updated image with the new packages required for running the new environment settings:

```
cd Ros-2-Environment/ros2_env/
./build_docker.sh

```
After, run the created docker container after building:

```
cd Ros-2-Environment/ros2_env/docker/
./start_integrated.sh

```

Optional but recommended: if you want to install tmux for opening multiple terminal sessions inside the running container, run the below commands inside the container. From how the dockerfile is currently settled up, is required to install it every time you run the container. Therefore, if you quit the running container, tmux will be eliminated and requires to be install installed again. We can think about adding it into the dockerfile and then rebuild it.

```
cd ros2_ws/
sudo apt-get update
sudo apt install tmux

```

Now in a new tmux session, build, source to configure the environment so that the packages built within a specific workspace are accessible in the current terminal session and launch the drone model along with gazebo11 and the world:

```
colcon build
source install/setup.bash                           # execute this inside ros2_ws folder 
ros2 launch falco_drone_bringup falco_drone_bringup

```

After create a new tmux session, we need to run the node **drone_position_control.py** , which enables closed loop pose and velocities control for reaching a certain goal position through ```self.move_drone_to_pose(...)``` method.

```
source install/setup.bash
ros2 run falco_drone_control drone_position_control
```

Currently the PID control system is implemented through plugins loaded into gazebo. So by navigating into ```cd Ros-2-Environment/ros2_env/ros2_ws/src/falco_drone/falco_drone_description/``` we will find the **plugin_drone_private.cpp** file, which is essential for:

- Loading the PID parameters from the sdf file
- Developing a series of callbacks for reading sensor data, cmd_vel and pose data. In addition, the velocity commands models sensor noise. Furhter callbacks are implemented for gathering the drone state and perform landing and takeoff operations.
- Implementing the publish odometry method, which is used to publish the drone state in the form of a nav_msgs/Odometry message. Notice that are published the odometry data in the base_footprint frame, which is the frame of reference for the drone's position and orientation in the world (odom). Along with the odometry data, the drone's pose is also published in the tf tree, which is essential for visualizing the drone's position and orientation in the Gazebo simulation environment.
- Implementing the UpdateState method, which handles the identification of the current state of the drone. This includes determining whether the drone is in a flying, landing, or taking off state.
- Implementing the UpdateDynamics method, which is responsible for updating the drone's dynamics based on the current state and the PID control inputs. This method applies the PID control outputs to the drone's motors to achieve the desired flight behavior.

In **drone_position_control.py**, we can manage to control the drone in high level, by setting target positions to be reached. It is also been added a safety mechanism that allows the drone to land safely if it is colse to the targed altitude, which is controlled by the ```self.landing_timer```. This timer is set to trigger a landing operation if the drone has a vertical distance with respect to the gournd close to the targed altitude.

**ADDITIONAL INFORMATION**

If you want to modify the drone's parameters (e.g. mass or inertia), you should edit the file **falco_drone.urdf.xacro**. The latter file is located in the urdf folder. 

After the parameters are modified, you need to run the updated .xacro file as follows:

```
cd /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws/src/falco_drone/falco_drone_description/urdf/
ros2 run xacro xacro falco_drone.urdf.xacro > falco_drone.urdf
```

The latter command generates a new .urdf file that incorporates the changes made in the .xacro file.
Then, a new .sdf file needs to be generated in order to incorporate the changes made in the .urdf file. This can be done running the following command:

```
cd /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws/src/falco_drone/falco_drone_description/models/falco_drone/
gz sdf -p ../../urdf/falco_drone.urdf > falco_drone.sdf
```
