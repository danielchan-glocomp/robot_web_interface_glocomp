# robot_web_interface_glocomp

### How to run
First startup a ros_bridge_websocket

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then in another terminal and run

```
cd robot_web_interface_glocomp/robot_web_interface
python3 -m http.server 8000
```

### Features
> - General robot status
> - Battery percentage
> - Front and back camera streams
> - Teleop control
> - Smartphone and computer support
> - /map topic is being displayed and robot position is being displayed

### ToDo
> - Change camera stream from ros message to webrtc
> - Get robot TFs displayed
> - Intergration with slam on the robot
> - map interaction
source /opt/ros/humble/setup.bash