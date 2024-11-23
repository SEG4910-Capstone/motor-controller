# Motor Controller

Custom hardware interface for the HDC2460 motor controller.

It's purpose is to translate commands from ros2_control written in C++ into a format that the motor controller understands, which in this case is in serial format.

This custom interface has been adapted from the [ros2_control_demo example respository](https://github.com/ros-controls/ros2_control_demos/tree/humble/example_2) as well as an [example interface for an Arduino machine](https://github.com/joshnewans/diffdrive_arduino/tree/humble), using commands defined in the [HDC2460 manual](https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file)


## Usage

Build
```
colcon build --packages-select snowplow_motor_controller --symlink-install
```

Launch urdf file 
```
ros2 launch snowplow_motor_controller diffbot.launch.py
```

Launch teleop twist keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diffbot_base_controller/cmd_vel   
```

