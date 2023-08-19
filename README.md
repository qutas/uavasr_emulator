# uavasr_emulator
Unmanned Aerial Vechicle Artic Search and Rescue Emulator. Small ROS publisher to output dummy data in the form of images, pose messages, and sensor readings.

## Installing Dependencies
Please download and install using the instructions for the [QUT Flight Stack](https://github.com/qutas/info/wiki/UAV-Setup-Guides-(2023)#the-qutas-flight-stack)

## Running

```sh
roslaunch uavasr_emulator emulator.launch
```

Additionally, a unique UAV name can be used when launching the emulator (replace `UAVNAME` with the desired name, defaults to "uavasr"):
```sh
roslaunch uavasr_emulator emulator.launch uav_name:=UAVNAME
```

#### Other Launch Files
- If you have the `spar` packages installed, you can find details to launching the combined system on the [Spar Readme](https://github.com/qutas/spar/).
- If you have the `qutas_lab_450` packages installed, you can find details to launching a full "assessment system" on the [QUTAS Lab 450 Readme](https://github.com/qutas/qutas_lab_450/).

## Subscribed Topics
- Local Position Setpoint ([mavros\_msgs/PositionTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html)): `/UAVNAME/setpoint_raw/local`

## Published Topics
- Autopilot State ([mavros\_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html)): `/UAVNAME/state`
- Battery Reading ([sensor\_msgs/BatteryState](http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html)): `/UAVNAME/battery`
- Current Position ([geometry\_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)): `/UAVNAME/pose`
- Current Odometery ([nav\_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)): `/UAVNAME/odom`
- Local Position Setpoint Feedback ([mavros\_msgs/PositionTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html)): `/UAVNAME/setpoint_raw/local`
