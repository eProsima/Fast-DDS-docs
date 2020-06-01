.. _typical_use_cases:

Typical Use-Cases
=================

.. _OSRF: https://www.openrobotics.org/
.. _Robot Operation System 2 (ROS 2): https://index.ros.org/doc/ros2/

Fast DDS is highly configurable, which allows to use it in a large number of scenarios.
This section provides configuration examples for the following typical use cases when dealing
with distributed systems:

+ :ref:`use-case-fast-rtps-over-wifi`.
  Presents a case where :ref:`discovery` through multicast communication is a challenge.
  This example shows how to:

  - configure an initial list of peers with the address-port pairs of the remote participants
    (see :ref:`use-case-initial-peers`).

  - disable the multicast discovery mechanism (see :ref:`use-case-disabling-multicast-discovery`).

+ :ref:`wide_deployments`.
  Describes a situation with a high number of deployed communicating agents.
  In these kind of environments, Fast DDS recommends using one of the following configurations:

  - the use of a centralized server for the :ref:`discovery` phases (see :ref:`server-client-discovery-use-case`).

  - the STATIC :ref:`discovery` mechanism for well known network topologies (see :ref:`wide_deployments_static`).

+ :ref:`use-case-manySubscribers`.
  In cases where there are many :ref:`DataReaders<dds_layer_subscriber_dataReader>` subscribed to the same
  :ref:`dds_layer_topic_topic`, using multicast delivery can help reducing the overhead in the network and CPU.

+ :ref:`fastrtps_ros2`.
  Since Fast DDS is the default middleware implementation in the `OSRF`_ `Robot Operation System 2 (ROS 2)`_,
  this tutorial is an explanation of how to take full advantage of Fast DDS wide set of capabilities in a ROS 2
  project.

.. toctree::
    :maxdepth: 2

    /fastdds/use_cases/wifi/wifi.rst
    /fastdds/use_cases/wide_deployments/wide_deployments.rst
    /fastdds/use_cases/large_data/large_data.rst
    /fastdds/use_cases/many_subscribers/many_subscribers.rst
    /fastdds/use_cases/ros2/ros2.rst



