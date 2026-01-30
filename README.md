# robot_web_interface_glocomp

### How to run
First startup a ros_bridge_websocket

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then in another terminal and run

```
cd robot_web_interface_glocomp
python3 -m https.server 8000
```

### Features
> - General robot status
> - Battery percentage
> - Front and back camera streams
> - Teleop control
> - Smartphone and computer support

### ToDo
> - Change camera stream from ros message to webrtc
> - Get map and robot TFs displayed
> - Intergration with slam on the robot
