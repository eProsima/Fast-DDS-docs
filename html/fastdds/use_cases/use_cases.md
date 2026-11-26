<a id="use-cases"></a>

<a id="id1"></a>

# Use-Cases

*Fast DDS* is highly configurable, which allows for its use in a large number of scenarios.
This section provides configuration examples for the following typical use cases when dealing
with distributed systems:

+ [DDS-TSN](tsn/tsn.md#use-case-dds-tsn) **[Pro]**.
  Describes how to configure *Fast DDS* to use DDS over Time-Sensitive Networking (TSN).
  This configuration leverages IEEE 802.1 standards to provide deterministic, real-time data delivery
  over Ethernet.
  It ensures predictable latency and high reliability for critical applications by mapping DDS QoS policies
  to TSN capabilities and supporting both UDP/IP and raw Ethernet transports.
+ [IP mobility](ip_mobility/ip_mobility.md#ip-mobility) **[Pro]**.
  If the network interfaces are expected to change while the application is running, *Fast DDS Pro* provides an
  automatic way of re-scanning the available interfaces and including them.
+ [TCP Keep Alive](ip_mobility/ip_mobility.md#tcp-keep-alive) **[Pro]**.
  In the same line as [IP mobility](ip_mobility/ip_mobility.md#ip-mobility), TCP Keep Alive feature allows to automatically detect
  broken TCP connections by sending periodic keep-alive messages at application level when the connection is idle.
  This feature is essential in TCP scenarios with dynamic IP changes or unstable network conditions,
  ensuring that stale connections are identified and re-established promptly.
+ [Large Data mode and Fast DDS over TCP](tcp/tcp_use_case.md#use-case-tcp).
  Describes how to configure *Fast DDS* to use the `LARGE_DATA` builtin transports mode.
  This mode enables efficient utilization of TCP transport without the need for constant reconfiguration
  during deployment changes.
  It optimizes communication performance for large data samples over lossy networks by employing a combination
  of UDP and TCP/SHM transports.
+ [Fast DDS over WIFI](wifi/wifi.md#use-case-fast-rtps-over-wifi).
  Presents a case where [Discovery](../discovery/discovery.md#discovery) through multicast communication is a challenge.
  This example shows how to:
  - Configure an initial list of peers with the address-port pairs of the remote participants
    (see [Configuring Initial Peers](wifi/initial_peers.md#use-case-initial-peers)).
  - Disable the multicast discovery mechanism (see [Disabling multicast discovery](wifi/disable_multicast.md#use-case-disabling-multicast-discovery)).
  - Configure a SERVER discovery mechanism (see [Discovery Server](wifi/discovery_server_use_case.md#discovery-server-use-case)).
+ [Well Known Network Deployments](well_known_deployments/well_known_deployments.md#well-known-deployments).
  Describes a situation where the entire entity network topology (Participants, Publishers, Subscribers,
  and their addresses and ports) are known beforehand.
  In these kind of environments, *Fast DDS* allows to completely avoid the discovery phase
  configuring a STATIC discovery mechanism.
+ [Topics with many subscribers](many_subscribers/many_subscribers.md#use-case-manysubscribers).
  In cases where there are many [DataReaders](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) subscribed to the same
  [Topic](../dds_layer/topic/topic/topic.md#dds-layer-topic-topic), using multicast delivery can help reducing the overhead in the network and CPU.
+ [Large Data Rates](large_data/large_data.md#use-case-largedata).
  Presents configuration options that can improve the performance in scenarios where the amount of data exchanged
  between a [Publisher](../dds_layer/publisher/publisher.md#dds-layer-publisher) and a [Subscriber](../dds_layer/subscriber/subscriber.md#dds-layer-subscriber) is large, either because of the data size
  or because the message rate.
  The examples describe how to:
  - Use TCP based communications (see [Large Data mode and Fast DDS over TCP](tcp/tcp_use_case.md#use-case-tcp)).
  - Configure the socket buffer size (see [increase the buffers size](large_data/large_data.md#tuning-socket-buffer)).
  - Limit the publication rate (see [Flow Controllers](large_data/large_data.md#flow-controllers)).
  - Tune the size of the socket buffers (see [Increasing socket buffers size](large_data/large_data.md#tuning-socket-buffer)).
  - Tune the Heartbeat period (see [Tuning Heartbeat Period](large_data/large_data.md#tuning-heartbeat-period)).
  - Configure a non-strict reliable mode (see [Using Non-strict Reliability](large_data/large_data.md#tuning-nonstrict-reliability)).
+ [Real-time behavior](realtime/realtime.md#use-case-realtime).
  Describes the configuration options that allows using *Fast DDS* on a real-time scenario.
  The examples describe how to:
  - Configure memory management to avoid dynamic memory allocation (see [Tuning allocations](realtime/allocations.md#realtime-allocations)).
  - Limit the blocking time of API functions to have a predictable response time (see [Non-blocking calls](realtime/blocking.md#non-blocking-calls)).
+ [Reduce memory usage](reduce_memory/reduce_memory.md#use-case-reduce-memory).
  For use cases with memory consumption constraints, *Fast DDS* can be configured to reduce memory footprint
  to a minimum by adjusting different QoS policies.
+ [Zero-Copy communication](zero_copy/zero_copy.md#use-case-zero-copy).
  Under certain constraints, *Fast DDS* can provide application level communication
  between publishing and subscribing nodes avoiding any data copy during the process.
+ [Unique network flows](unique_network_flows/unique_network_flows.md#use-case-unique-flows).
  This use case illustrates the APIs that allow for the request of unique network flows, and for the identification
  of those in use.
+ [Statistics module](statistics_module/statistics_module.md#statistics-module).
  This use case explains how to enable the Statistics module within the monitored application, and how to create a
  statistics monitoring application.
+ [ROS 2 using Fast DDS middleware](../ros2/ros2.md#ros2).
  Since *Fast DDS* is the default middleware implementation in every [OSRF](https://www.openrobotics.org/) [Robot Operation System 2 (ROS 2)](https://index.ros.org/doc/ros2/)
  long term (LTS) releases and most of the non-LTS releases,
  this documentation includes a whole independent section to show the use of the library in ROS 2,
  and how to take full advantage of *Fast DDS* wide set of capabilities in a ROS 2 project.
+ [How to use eProsima DDS Record and Replay](dds_record_and_replay/dds_record_and_replay.md#dds-record-and-replay).
  Instructions on how to tune your application to be able to record and replay your DDS messages using
  [eProsima DDS Record and Replay](https://github.com/eProsima/DDS-Record-Replay).
+ [Request-Reply communication](request_reply/request_reply.md#use-case-request-reply).
  Although *Fast DDS* provides a *Publisher-Subscriber* communication protocol, it offers a mechanism to use
  *Request-Reply* communications.
  This use case explains how to use the APIs to communicate two application using *Request-Reply*.
+ [Remote type discovery and endpoint matching](remote_type_discovery_matching/remote_type_discovery_matching.md#use-case-remote-type-discovery-and-matching).
  This use case leverages the [Remote Data Types Discovery](../xtypes/discovery_matching.md#xtypes-discovery-matching) to create endpoints and start communicating with remote
  discovered endpoints with previously unknown data types.
+ [Custom buffers](custom_buffers/custom_buffers.md#use-case-custom-buffers)  **[Pro]**.
  Buffers of the unbounded sequences of octets of a data type may keep their data out of the CPU memory
  (e.g. in a GPU or in a file).
  This use case explains how to implement a custom buffer and a backend that decides what is sent in the place of its
  data, and how to use them in a DataWriter and a DataReader.
