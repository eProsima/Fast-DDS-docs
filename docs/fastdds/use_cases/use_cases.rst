.. _typical_use_cases:

Typical Use-Cases
=================

.. _OSRF: https://www.openrobotics.org/
.. _Robot Operation System 2 (ROS 2): https://index.ros.org/doc/ros2/

*Fast DDS* is highly configurable, which allows for its use in a large number of scenarios.
This section provides configuration examples for the following typical use cases when dealing
with distributed systems:

+ :ref:`use-case-fast-rtps-over-wifi`.
  Presents a case where :ref:`discovery` through multicast communication is a challenge.
  This example shows how to:

  - Configure an initial list of peers with the address-port pairs of the remote participants
    (see :ref:`use-case-initial-peers`).

  - Disable the multicast discovery mechanism (see :ref:`use-case-disabling-multicast-discovery`).

  - Configure a SERVER discovery mechanism (see :ref:`server-client-discovery-use-case`).

+ :ref:`well_known_deployments`.
  Describes a situation where the entire entity network topology (Participants, Publishers, Subscribers,
  and their addresses and ports) are known beforehand.
  In these kind of environments, *Fast DDS* allows to completely avoid the discovery phase
  configuring a STATIC discovery mechanism.

+ :ref:`use-case-manySubscribers`.
  In cases where there are many :ref:`DataReaders<dds_layer_subscriber_dataReader>` subscribed to the same
  :ref:`dds_layer_topic_topic`, using multicast delivery can help reducing the overhead in the network and CPU.

+ :ref:`use-case-largeData`.
  Presents configuration options that can improve the performance in scenarios where the amount of data exchanged
  between a :ref:`dds_layer_publisher` and a :ref:`dds_layer_subscriber` is large, either because of the data size
  or because the message rate.
  The examples describe how to:

  - Configure the socket buffer size (see :ref:`increase the buffers size<tuning-socket-buffer>`).

  - Limit the publication rate (see :ref:`flow-controllers`).

  - Tune the size of the socket buffers (see :ref:`tuning-socket-buffer`).

  - Tune the Heartbeat period (see :ref:`tuning-heartbeat-period`).

  - Configure a non-strict reliable mode (see :ref:`tuning-nonstrict-reliability`).

+ :ref:`use-case-realtime`.
  Describes the configuration options that allows using *Fast DDS* on a real-time scenario.
  The examples describe how to:

  - Configure memory management to avoid dynamic memory allocation (see :ref:`realtime-allocations`).

  - Limit the blocking time of API functions to have a predictable response time (see :ref:`non-blocking-calls`).

+ :ref:`fastrtps_ros2`.
  Since *Fast DDS* is the default middleware implementation in the `OSRF`_ `Robot Operation System 2 (ROS 2)`_,
  this tutorial is an explanation of how to take full advantage of *Fast DDS* wide set of capabilities in a ROS 2
  project.

.. toctree::
    :maxdepth: 2

    /fastdds/use_cases/wifi/wifi.rst
    /fastdds/use_cases/well_known_deployments/well_known_deployments.rst
    /fastdds/use_cases/large_data/large_data.rst
    /fastdds/use_cases/many_subscribers/many_subscribers.rst
    /fastdds/use_cases/realtime/realtime.rst
    /fastdds/use_cases/ros2/ros2.rst



