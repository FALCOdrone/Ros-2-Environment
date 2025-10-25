## INSTRUCTIONS FOR RUNNING A NEW CUSTOM X500 DRONE SIMULATION WITH ROS2 BRIDGE

1. Build the docker image (if not done already):

   ```bash
   cd drone_control_ws/docker
   docker build -t drone_control_image .
   ```
2. Run the docker container:

   ```bash
   docker-compose up -d drone_control_container
   docker exec -it drone_control_container bash
   ```
3. Inside the container, launch the Gazebo simulation:

   ```bash
   ./launch_x500_enhanced.sh

    # Keep this terminal running to maintain the simulation environment
    ```
4. Open a new terminal and attach to the running container:

   ```bash
   docker exec -it drone_control_container bash
   ```
5. Inside the container, start the Gazebo-ROS2 bridge:

   ```bash
   ./gz_ros2_bridge.sh
    # Keep this terminal running to maintain the topic translations
    ```
6. Open another terminal and attach to the running container:

   ```bash
   docker exec -it drone_control_container bash
   ```
7. Now you can run your custom ROS2 nodes to control the drone, process sensor data, or visualize information. For example:

   ```bash
    # Your navigation/control code
    ros2 run your_package your_node
    # Or visual odometry
    ros2 launch stereo_slam stereo_slam.launch.py
    # Or view camera feeds
    ros2 run rqt_image_view rqt_image_view
    ```

### WHAT DOES THIS NEW X500 DRONE MODEL CONTAIN AND HOW TO VERIFY THE ROS2 BRIDGE IS WORKING?

The new `x500_enhanced` drone model includes the following sensors:
- GPS Sensor
- Stereo Cameras (Left and Right)
- IMU Sensor

To verify that the ROS2 bridge is functioning correctly and that data is flowing from Gazebo to ROS2, you can check the topics being published. Here are some example commands:

```bash
# Before listing the ros2 topics, check which topics are available inside gazebo
gz topic -l
```

```bash
# List all topics seen by ROS2 through the bridge
ros2 topic list

# Echo a specific topic
ros2 topic echo /your_topic_name