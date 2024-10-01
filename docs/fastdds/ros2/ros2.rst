.. _ros2:

ROS 2 using Fast DDS middleware
===============================

.. _OSRF: https://www.openrobotics.org/
.. _Robot Operation System 2 (ROS 2): https://index.ros.org/doc/ros2/

*Fast DDS* is the default middleware implementation in the
`Open Source Robotic Fundation (OSRF) <https://www.openrobotics.org/>`_
`Robot Operating System ROS 2 <https://index.ros.org/doc/ros2/>`_ in every long term (LTS) releases and most of the
non-LTS releases.

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
The default ROS 2 RMW implementation in all distributions except *EOL Galactic* is ``rmw_fastrtps_cpp``.
For *EOL Galactic* the environment variable ``RMW_IMPLEMENTATION`` has to be set to select ``rmw_fastrtps_cpp`` in order to
use *Fast DDS* as the middleware layer.
This environment variable can also be used to select the ``rmw_fastrtps_dynamic_cpp`` implementation:

#. Exporting ``RMW_IMPLEMENTATION`` environment variable:

   ::

       export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

   or

   ::

       export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp

#. When launching your ROS 2 application:

   ::

       RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run <package> <application>

   or

   ::

       RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp ros2 run <package> <application>

.. note::

   On *EOL Galactic* you may have to install the ``rmw_fastrtps_cpp`` package:

   .. code-block:: bash

      sudo apt install ros-galactic-rmw-fastrtps-cpp

.. toctree::
    :maxdepth: 1

    /fastdds/ros2/ros2_configure.rst
    /fastdds/ros2/discovery_server/ros2_discovery_server.rst
