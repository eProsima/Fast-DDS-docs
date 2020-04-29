The use of Fast-RTPS is highly varied, allowing a large number of configurations depending on the scenario in which
the library is applied.
This section provides configuration examples for the typical use cases arousing when dealing
with distributed systems.
It is organized as follows:

+ :ref:`use-case-fast-rtps-over-wifi`.
  Presents the case of using Fast-RTPS in scenarios where discovery through multicast communication is a challenge.
  To address this problem, the use of an initial peers list by which the
  address-port pairs of the remote participants are defined is presented (See :ref:`use-case-initial-peers`).
  Furthermore, it specifies how to disable the multicast discovery mechanism (See
  :ref:`use-case-disabling-multicast-discovery`).
+ :ref:`wide_deployments`.
  Describes the recommended configurations for using Fast-RTPS in environments with a high
  number of deployed communicating agents.
  These are the use of a centralized server for the discovery phases (See :ref:`server-client-discovery-use-case`), and
  the Fast-RTPS' STATIC discovery mechanism for well known network topologies (See :ref:`wide_deployments_static`).
+ :ref:`fastrtps_ros2`.
  Since Fast-RTPS is the default middleware implementation in the
  `OSRF <https://www.openrobotics.org/>`_ `Robot Operation System 2 (ROS 2) <https://index.ros.org/doc/ros2/>`_,
  this tutorial is an explanation of how to take full advantage of Fast-RTPS wide set of capabilities in a ROS 2
  project.

