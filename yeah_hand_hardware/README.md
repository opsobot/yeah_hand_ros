# yeah_hand_hardware_interface

   The example shows how to implement robot hardware with separate communication to each actuator.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_6/doc/userdoc.html).


# Set up bluetooth pairing and device file
sudo rfcomm bind 0 94:51:DC:2D:B5:A2 1
rfcomm -a
ls -l /dev/rfcomm0
sudo chmod 666 /dev/rfcomm0

