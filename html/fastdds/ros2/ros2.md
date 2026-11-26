<a id="ros2"></a>

<a id="ros-2-using-fast-dds-middleware"></a>

# ROS 2 using Fast DDS middleware

*Fast DDS* is the default middleware implementation in the
[Open Source Robotic Fundation (OSRF)](https://www.openrobotics.org/)
[Robot Operating System ROS 2](https://index.ros.org/doc/ros2/) in every long term (LTS) releases and most of the
non-LTS releases.

**ROS 2** is a state-of-the-art software for robot engineering which
consists of a set of [free software libraries](https://github.com/ros2) and tools for building robot applications.
This section presents some use cases and shows how to take full advantage of
Fast DDS wide set of capabilities in a ROS 2 project.

The interface between the ROS 2 stack and *Fast DDS* is provided by a ROS 2 package
[rmw_fastrtps](https://github.com/ros2/rmw_fastrtps).
This package is available in all ROS 2 distributions, both from binaries and from sources.
`rmw_fastrtps` actually provides not one but two different ROS 2 middleware implementations, both of them using *Fast
DDS* as middleware layer: `rmw_fastrtps_cpp` and `rmw_fastrtps_dynamic_cpp`.
The main difference between the two is that `rmw_fastrtps_dynamic_cpp` uses introspection type support at run time to
decide on the serialization/deserialization mechanism, while `rmw_fastrtps_cpp` uses its own type support, which
generates the mapping for each message type at build time.
The default ROS 2 RMW implementation in all distributions except *EOL Galactic* is `rmw_fastrtps_cpp`.
For *EOL Galactic* the environment variable `RMW_IMPLEMENTATION` has to be set to select `rmw_fastrtps_cpp` in
order to use *Fast DDS* as the middleware layer.
This environment variable can also be used to select the `rmw_fastrtps_dynamic_cpp` implementation:

1. Exporting `RMW_IMPLEMENTATION` environment variable:
   ```default
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

   or
   ```default
   export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp
   ```
2. When launching your ROS 2 application:
   ```default
   RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run <package> <application>
   ```

   or
   ```default
   RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp ros2 run <package> <application>
   ```

#### NOTE
On *EOL Galactic* you may have to install the `rmw_fastrtps_cpp` package:

```bash
sudo apt install ros-galactic-rmw-fastrtps-cpp
```

* [Configuring *Fast DDS* in ROS 2](ros2_configure.md)
* [Use ROS 2 with Fast-DDS Discovery Server](discovery_server/ros2_discovery_server.md)
