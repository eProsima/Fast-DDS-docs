.. _ros2:

ROS 2 using Fast DDS middleware
===============================

.. _OSRF: https://www.openrobotics.org/
.. _Robot Operation System 2 (ROS 2): https://index.ros.org/doc/ros2/

*Fast DDS* is the default middleware implementation in the
`Open Source Robotic Fundation (OSRF) <https://www.openrobotics.org/>`_
`Robot Operating System ROS 2 <https://index.ros.org/doc/ros2/>`_.

**ROS 2** is a state-of-the-art software for robot engineering which
consists of a set of `free software libraries <https://github.com/ros2>`__ and tools for building robot applications.
This section presents some use cases and shows how to take full advantage of
Fast DDS wide set of capabilities in a ROS 2 project.

The interface between the ROS 2 stack and *Fast DDS* is provided by a ROS 2 package
`rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`_.
This package is available in all ROS 2 distributions, both from binaries and from sources.
``rmw_fastrtps`` actually provides not one but two different ROS 2 middleware implementations, both of them using *Fast
DDS* as middleware layer: ``rmw_fastrtps_cpp`` and ``rmw_fastrtps_dynamic_cpp``.
The main difference between the two is that ``rmw_fastrtps_dynamic_cpp`` uses introspection type support at run time to
decide on the serialization/deserialization mechanism, while ``rmw_fastrtps_cpp`` uses its own type support, which
generates the mapping for each message type at build time.
The default ROS 2 RMW implementation is ``rmw_fastrtps_cpp``.
However, it is still possible to select ``rmw_fastrtps_dynamic_cpp`` by using the environment variable
``RMW_IMPLEMENTATION``:

#. Exporting ``RMW_IMPLEMENTATION`` environment variable:

   ::

       export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp

#. When launching your ROS 2 application:

   ::

       RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp ros2 run <package> <application>


.. toctree::
    :maxdepth: 1

    /fastdds/ros2/ros2_configure.rst
    /fastdds/ros2/Discovery_server/ros2_discovery_server.rst
