.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _fastrtps_ros2:

Fast DDS in ROS 2
==================

*Fast DDS* is the default middleware implementation in the
`Open Source Robotic Fundation (OSRF) <https://www.openrobotics.org/>`_
`Robot Operating System ROS 2 <https://index.ros.org/doc/ros2/>`_.
This tutorial is an explanation of how to take full advantage of *Fast DDS* wide set of capabilities
in a ROS 2 project.

The interface between the ROS2 stack and *Fast DDS* is provided by a ROS 2 package
`rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`_.
This package is available in all ROS 2 distributions, both from binaries and from sources.
``rmw_fastrtps`` actually provides not one but two different ROS 2 middleware implementations, both of them using
*Fast DDS* as middleware layer: ``rmw_fastrtps_cpp`` and ``rmw_fastrtps_dynamic_cpp``.
The main difference between the two is that ``rmw_fastrtps_dynamic_cpp`` uses introspection type support at run time to
decide on the serialization/deserialization mechanism, while ``rmw_fastrtps_cpp`` uses its own type support, which
generates the mapping for each message type at build time.
The default ROS 2 RMW implementation is ``rmw_fastrtps_cpp``.
However, it is still possible to select ``rmw_fastrtps_dynamic_cpp`` using the environment variable
``RMW_IMPLEMENTATION``:

#. Exporting ``RMW_IMPLEMENTATION`` environment variable:

   ::

       export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp

#. When launching your ROS 2 application:

   ::

       RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp ros2 run <package> <application>

.. _ros2_use_xml:

Configuring Fast DDS with XML files
-------------------------------------

As described in :ref:`xml_profiles` section, there are two possibilities for providing *Fast DDS*
with XML configuration files:

* **Recommended**: Define the location of the XML configuration file with environment variable
  ``FASTRTPS_DEFAULT_PROFILES_FILE`` (see :ref:`env_vars`).

  ::

      export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>

* **Alternative**: Create a *DEFAULT_FASTRTPS_PROFILES.xml* and place it in the same directory as the application
  executable.

Default profiles
^^^^^^^^^^^^^^^^

Under ROS 2, the entity creation does not allow for selecting different profiles from the XML.
To work around this issue, the profiles can be marked with an attribute ``is_default_profile="true"``, so when an entity
of that type is created, it will automatically load that profile.
The mapping between ROS 2 entities and *Fast DDS* entities is:

+--------------+------------------------+
| ROS entity   | *Fast DDS* entity      |
+==============+========================+
| Node         | Participant            |
+--------------+------------------------+
| Publisher    | Publisher              |
+--------------+------------------------+
| Subscription | Subscriber             |
+--------------+------------------------+
| Service      | Publisher + Subscriber |
+--------------+------------------------+
| Client       | Publisher + Subscriber |
+--------------+------------------------+

For example, a profile for a ROS 2 ``Node`` would be specified as:

+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
|    :language: xml                                       |
|    :start-after: <!-->CONF_ROS2_DEFAULT_PROFILE         |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

Configure Publication Mode and History Memory Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, ``rmw_fastrtps`` sets some of the *Fast DDS* configurable parameters, ignoring whatever configuration is
provided in the XML file.
Said parameters, and their default values under ROS 2, are:

.. list-table::
   :header-rows: 1
   :align: left

   * - Parameter
     - Description
     - Default ROS 2 value
   * - |MemoryManagementPolicy|
     - *Fast DDS* preallocates memory for the publisher |br|
       and subscriber histories. When those histories fill |br|
       up, a reallocation occurs to reserve more memory.
     - |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|
   * - |PublishModeQosPolicy|
     - User calls to publication method add the messages |br|
       in a queue that is managed in a different thread, |br|
       meaning that the user thread is available right |br|
       after the call to send data.
     - |ASYNCHRONOUS_PUBLISH_MODE-api|


However, it is possible to fully configure *Fast DDS* (including the history memory policy and the publication mode)
using an XML file in combination with environment variable ``RMW_FASTRTPS_USE_QOS_FROM_XML``.

::

    export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
    ros2 run <package> <application>

.. _ros2_example:

Example
-------

The following example uses the ROS 2 talker/listener demo, configuring *Fast DDS* to publish synchronously,
and to have a dynamically allocated publisher and subscriber histories.

#. Create a XML file `ros_example.xml` and save it in `path/to/xml/`

   +---------------------------------------------------------+
   | **XML**                                                 |
   +---------------------------------------------------------+
   | .. literalinclude:: /../code/XMLTester.xml              |
   |    :language: xml                                       |
   |    :start-after: <!-->CONF_ROS2_EXAMPLE                 |
   |    :end-before: <!--><-->                               |
   +---------------------------------------------------------+

#. Open one terminal and run:

   ::

       export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
       export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/xml/ros_example.xml
       export RMW_FASTRTPS_USE_QOS_FROM_XML=1
       ros2 run demo_nodes_cpp talker

#. Open one terminal and run:

   ::

       export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
       export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/xml/ros_example.xml
       export RMW_FASTRTPS_USE_QOS_FROM_XML=1
       ros2 run demo_nodes_cpp listener
