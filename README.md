libcopter_ros
=============

This ROS module makes the [libcopter](https://windfis.ch/libcopter/libcopter.html)
([@github](https://github.com/Windfisch/libcopter)) functionality available to
ROS clients.

libcopter supports various similar cheap chinese copters that are controlled via WiFi and have a camera on board.
Currently, the SG500 and the JJRC blue crab drones are supported.

[ROS (short for Robot Operating System)](https://www.ros.org/) describe themselves as "a set of software libraries and tools that help you build robot applications".

Messages and usage
------------------

For an initial setup, do this:

```
cd ros-workspace/src
git clone --recursive https://github.com/Windfisch/libcopter_ros
```

After building your workspace, you can launch it using `rosrun libcopter_ros libcopter_ros_node`.

*libcopter_ros_node* will then emit and listen to the following messages:

  - constantly emits a **camera_raw** message of type `sensor_msgs::Image`, which contains the current image from the drone's video stream
  - the **telemetry1** and **telemetry2** streams (which are somewhat misnamed) emit the (yet unknown) additional data coming from the drone
  - listens on the **cmd_vel** topic, expecting `geometry_msgs::Twist` messages to control the drone
  - sending an `std_msgs::Empty` message to either the **takeoff**, **land** or **panic** topics will perform an automatic takeoff, automatic landing or an immediate motor shut down action.


