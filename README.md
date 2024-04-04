# Motor Controller

Custom hardware interface for the HDC2460 motor controller.

It's purpose is to translate commands from ros2_control written in C++ into a format that the motor controller understands, which in this case is in serial format.

This custom interface has been adapted from the [ros2_control_demo example respository](https://github.com/ros-controls/ros2_control_demos/tree/humble/example_2) as well as an [example interface for an Arduino machine](https://github.com/joshnewans/diffdrive_arduino/tree/humble), using commands defined in the [HDC2460 manual](https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file)
