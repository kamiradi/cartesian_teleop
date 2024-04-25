# Cartesian Teleoperation of a robot

This repository aims to be a utility sitting on top of a ROS1 cartesian
impedance controller exposing a ros topic interface to a `equilibrium_pose`.

It was written for a spacemouse, but supports any joystick device that exposes
the relevant `sensor_msgs.Joy` topic
