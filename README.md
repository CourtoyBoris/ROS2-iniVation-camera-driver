# ROS2 iniVation camera driver

ROS2 packages managing the integration of the event-based cameras from iniVation in a ROS2 project.
This package has been developped under ROS2 Galactic (https://docs.ros.org/en/galactic/index.html) and relies on the dv-processing library fromm iniVation (https://dv-processing.inivation.com/rel_1_7/index.html). Make sure to install them properly.
Because of C++ incompatibilities, we have to sligthly modify the ROS2 distribution to make it C++20 friendly, as the dv-processing relies on C++20 mechanisms. The modification can be found in the following github issue : (https://github.com/ros2/rclcpp/pull/1678)

## Compilation
You first need to compile the message package before compiling the camera one. You must compile using gcc-10, which can be done using specific flags (--cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10). The packages can be compiled in the following way :

```
colcon build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10 --packages-select message
```

```
colcon build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10 --packages-select camera
```

## Running nodes
Each node can be launched in the following way :

```
ros2 run camera "node_name"
```

With node_name being either "talker" , "event" or "frame". Talker is the node retrieving data from the camera and publishing them to the corresponding topics, while event and frame are the nodes reading data from those topics and displaying them.
