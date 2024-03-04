.. _typical_use_cases:

Typical Use-Cases
=================

.. _OSRF: https://www.openrobotics.org/
.. _Robot Operation System 2 (ROS 2): https://index.ros.org/doc/ros2/

*Fast DDS* is highly configurable, which allows for its use in a large number of scenarios.
This section provides configuration examples for the following typical use cases when dealing
with distributed systems:

+ :ref:`use-case-tcp`.
  Describes how to configure *Fast DDS* to use the ``LARGE_DATA`` builtin transports mode. This mode enables
  efficient utilization of TCP transport without the need for constant reconfiguration during deployment changes. It
  optimizes communication performance for large data samples over lossy networks by employing a combination of UDP and
  TCP/SHM transports.

+ :ref:`use-case-fast-rtps-over-wifi`.
  Presents a case where :ref:`discovery` through multicast communication is a challenge.
  This example shows how to:

  - Configure an initial list of peers with the address-port pairs of the remote participants
    (see :ref:`use-case-initial-peers`).

  - Disable the multicast discovery mechanism (see :ref:`use-case-disabling-multicast-discovery`).

  - Configure a SERVER discovery mechanism (see :ref:`discovery-server-use-case`).

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

  - Use TCP based communications (see :ref:`use-case-tcp`).

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

+ :ref:`use-case-reduce-memory`.
  For use cases with memory consumption constraints, *Fast DDS* can be configured to reduce memory footprint
  to a minimum by adjusting different QoS policies.

+ :ref:`use-case-zero-copy`.
  Under certain constraints, *Fast DDS* can provide application level communication
  between publishing and subscribing nodes avoiding any data copy during the process.

+ :ref:`use-case-unique-flows`.
  This use case illustrates the APIs that allow for the request of unique network flows, and for the identification
  of those in use.

+ :ref:`dynamic-network-interfaces`.
  If the network interfaces are expected to change while the application is running, *Fast DDS* provides an easy way
  of re-scanning the available interfaces and including them.

+ :ref:`statistics_module`.
  This use case explains how to enable the Statistics module within the monitored application, and how to create a
  statistics monitoring application.

+ :ref:`ros2`.
  Since *Fast DDS* is the default middleware implementation in every `OSRF`_ `Robot Operation System 2 (ROS 2)`_
  long term (LTS) releases and most of the non-LTS releases,
  this documentation includes a whole independent section to show the use of the library in ROS 2,
  and how to take full advantage of *Fast DDS* wide set of capabilities in a ROS 2 project.

+ :ref:`rosbag_capture`.
  Instructions on how to tune your application to be able to record and replay your DDS messages using ROS 2 rosbag2
  package.

.. toctree::
    :maxdepth: 2

    /fastdds/use_cases/wifi/wifi.rst
    /fastdds/use_cases/well_known_deployments/well_known_deployments.rst
    /fastdds/use_cases/large_data/large_data.rst
    /fastdds/use_cases/many_subscribers/many_subscribers.rst
    /fastdds/use_cases/realtime/realtime.rst
    /fastdds/use_cases/reduce_memory/reduce_memory.rst
    /fastdds/use_cases/zero_copy/zero_copy.rst
    /fastdds/use_cases/tcp/tcp_use_case.rst
    /fastdds/use_cases/unique_network_flows/unique_network_flows.rst
    /fastdds/use_cases/statistics_module/statistics_module.rst
    /fastdds/use_cases/dynamic_network_interfaces/dynamic_network_interfaces.rst
    /fastdds/use_cases/rosbag_capture/rosbag_capture.rst
