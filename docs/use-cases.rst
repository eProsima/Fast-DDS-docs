.. _typical_use_cases:

Typical Use-Cases
#################

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

.. _use-case-fast-rtps-over-wifi:

Fast-RTPS over WIFI
===================

The `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ defines the SIMPLE discovery as the default
mechanism for discovering participants in the network.
One of the main features of this mechanism is the use of multicast communication in the Participant Discovery
Phase (PDP).
This could be a problem in case the communication is not wired, i.e. WiFi communication, since multicast is
not as reliable over WiFi as it is over ethernet.
Fast-RTPS' solution to this challenge is to define the participants with which a unicast communication is to be set, i.e
an initial list of remote peers.

.. _use-case-initial-peers:

Initial Peers
-------------

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), each participant must
listen for incoming PDP discovery metatraffic in two different ports, one linked with a multicast address, and another
one linked to a unicast address.
Fast-RTPS allows for the configuration of an initial peers list which contains one or more such address-port pairs
corresponding to remote participants PDP discovery listening resources, so that the local participant will not only
send its PDP traffic to the default multicast address-port specified by its domain, but also to all the address-port
pairs specified in the :ref:`initial-peers` list.

A participant's initial peers list contains the list of address-port pairs of all other participants with
which it will communicate.
It is a list of addresses that a participant will use in the unicast discovery mechanism, together or as an alternative
to multicast discovery.
Therefore, this approach also applies to those scenarios in which multicast functionality is not available.

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the participants'
discovery traffic unicast listening ports are calculated using the following equation:
7400 + 250 * `domainID` + 10 + 2 * `participantID`.
Thus, if for example a participant operates in Domain 0 (default
domain) and its ID is 1, its discovery traffic unicast listening port would be: 7400 + 250 * 0 + 10 + 2 * 1 = 7412.

The following constitutes an example configuring an Initial Peers list with one peer on host 192.168.10.13 with
participant ID 1 in domain 0.

+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp              |
|    :language: c++                                       |
|    :start-after: //CONF_INITIAL_PEERS_BASIC             |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml               |
|    :language: xml                                       |
|    :start-after: <!-->CONF_INITIAL_PEERS_BASIC<-->      |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

.. _use-case-disabling-multicast-discovery:

Disabling multicast discovery
-----------------------------

If all the peers are known beforehand, it is possible to disable the multicast meta traffic completely.
This is done using the configuration attribute ``metatrafficUnicastLocatorList``.
By defining a custom ``metatrafficUnicastLocatorList``, the default metatraffic multicast and unicast locators to be
employed by the participant are avoided, which prevents the participant from listening to any discovery data from
multicast sources.
The local participant creates a meta traffic receiving resource per address-port pair specified in the
``metatrafficUnicastLocatorList``.

Consideration should be given to the assignment of the address-port pair in the ``metatrafficUnicastLocatorList``,
avoiding the assignment of ports that are not available or do not match the address-port
listed in the publisher participant Initial Peers list.

+------------------------------------------------------------+
| **C++**                                                    |
+------------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp                 |
|    :language: c++                                          |
|    :start-after: //CONF_INITIAL_PEERS_METAUNICAST          |
|    :end-before: //!--                                      |
+------------------------------------------------------------+
| **XML**                                                    |
+------------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml                  |
|    :language: xml                                          |
|    :start-after: <!-->CONF_INITIAL_PEERS_METAUNICAST<-->   |
|    :end-before: <!--><-->                                  |
+------------------------------------------------------------+

.. _wide_deployments:

Wide Deployments
================

Systems with large amounts of communication nodes might pose a challenge to
`Data Distribution Service (DDS) <https://www.omg.org/spec/DDS/1.4/PDF>`_ based middleware implementations in terms of
setup times, memory consumption, and network load.
This is because, as explained in :ref:`discovery`, the Participant Discovery Phase (PDP) relies on meta traffic
announcements sent to multicast addresses so that all the participants in the network can acknowledge each other.
This phase is followed by a Endpoint Discovery Phase (EDP) where all the participants exchange information (using
unicast addresses) about their publisher and subscriber entities with the rest of the participants, so that matching
between publishers and subscribers using the same topic can occur.
As the number of participants, publishers, and subscribers increases, the meta-traffic, as well as the number of
connections, increases exponentially, severely affecting the setup time and memory consumption.
Fast-RTPS provides extra features that expand the DDS standard to adapt it to wide deployment scenarios.

+-----------------------------------+---------------------------------------------------------------------------------+
| Feature                           | Purpose                                                                         |
+===================================+=================================================================================+
| Server-Client Discovery Mechanism | This feature is intended to substitute the standard SPDP and SEDP protocols     |
|                                   | with a discovery based on a server-client architecture, where all the           |
|                                   | meta-traffic goes through a hub (server) to be distributed throughout the       |
|                                   | network communication nodes.                                                    |
+-----------------------------------+---------------------------------------------------------------------------------+
| Static Discovery                  | With this feature, the user can manually specify which participant should       |
|                                   | communicate with which one and through which address and port.                  |
|                                   | Furthermore, the the user can specify which publisher/subscriber matches with   |
|                                   | which one, thus eliminating all EDP meta traffic.                               |
+-----------------------------------+---------------------------------------------------------------------------------+

.. _server-client-discovery-use-case:

Server-Client Discovery
-----------------------

Considering a scenario in which a large number of communication agents, called participants in this case, are deployed,
an alternative to the default RTPS standard SIMPLE discovery mechanism may be used.
For this purpose, Fast-RTPS
provides a client-server discovery mechanism, in which a server participant operates as the central point of
communication, that is the server collects and processes the metatraffic sent by the client participants, and
distributes the appropriate information among the rest of the clients.

Various discovery server use cases are presented below.

.. _discovery_server_major_scenario_setup:

UDPv4 example setup
^^^^^^^^^^^^^^^^^^^

To configure the client-server discovery scenario, two types of participants are created: the server participant and
the client participant.
Two parameters to be configured in this type of implementation are outlined:

+ **Prefix**: This is the unique identifier of the server.
+ **Address-port pair**: Specifies the IP address and port of the machine that implements the server.
  The port is a random number that can be replaced with any other value.
  Consideration should be given to the assignment of the address-port pair in the ``metatrafficUnicastLocatorList``,
  avoiding the assignment of ports that are not available.
  Thus using RTPS standard ports is discouraged.

+--------------------------------------------------------+--------------------------------------------------------+
| **SERVER**                                             | **CLIENT**                                             |
+--------------------------------------------------------+--------------------------------------------------------+
| **C++**                                                | **C++**                                                |
+--------------------------------------------------------+--------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp             | .. literalinclude:: ../code/CodeTester.cpp             |
|    :language: c++                                      |    :language: c++                                      |
|    :start-after: //CONF_DS_MAIN_SCENARIO_SERVER        |    :start-after: //CONF_DS_MAIN_SCENARIO_CLIENT        |
|    :end-before: //!--                                  |    :end-before: //!--                                  |
+--------------------------------------------------------+--------------------------------------------------------+
| **XML**                                                | **XML**                                                |
+--------------------------------------------------------+--------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml              | .. literalinclude:: ../code/XMLTester.xml              |
|    :language: xml                                      |    :language: xml                                      |
|    :start-after: <!-->CONF_DS_MAIN_SCENARIO_SERVER<--> |    :start-after: <!-->CONF_DS_MAIN_SCENARIO_CLIENT<--> |
|    :end-before: <!--><-->                              |    :end-before: <!--><-->                              |
+--------------------------------------------------------+--------------------------------------------------------+

.. _discovery_server_redundancy_scenario_setup:

UDPv4 redundancy example
^^^^^^^^^^^^^^^^^^^^^^^^

The :ref:`above example <discovery_server_major_scenario_setup>` presents a *single point of failure*, that is, if the
*server* fails there is no discovery. In order to prevent this, several servers could be linked to a *client*. By doing
this, a discovery failure only takes place if *all servers* fail, which is a more unlikely event.

The following values have been chosen in order to assure each server has a unique **Prefix** and *unicast address*:

.. csv-table::
    :header: "Prefix", "UDPv4 address"
    :widths: 20,100

    75.63.2D.73.76.72.63.6C.6E.74.2D.31, "192.168.10.57:56542"
    75.63.2D.73.76.72.63.6C.6E.74.2D.32, "192.168.10.60:56543"

.. image:: ds_redundancy.png
    :align: center
    :width: 75%

.. | @startuml
.. |
.. | package "Servers" {
.. |
.. | interface "\n192.168.10.57\n56542" as P1
.. | interface "\n192.168.10.60\n56543" as P2
.. |
.. | P1 -left- [75.63.2D.73.76.72.63.6C.6E.74.2D.31]
.. | P2 -left- [75.63.2D.73.76.72.63.6C.6E.74.2D.32]
.. |
.. | [75.63.2D.73.76.72.63.6C.6E.74.2D.31] -[hidden]up- [75.63.2D.73.76.72.63.6C.6E.74.2D.32]
.. | P1 -[hidden]up- P2
.. | }
.. |
.. | node "Clients" {
.. | (client\n1) as ps1
.. | (client\n2) as ps2
.. | (client\n3) as ps3
.. | (client\nX) as psX
.. | }
.. |
.. | ps1 -> P1
.. | ps1 .> P2
.. |
.. | ps2 -> P1
.. | ps2 .left.> P2
.. |
.. | ps3 -> P1
.. | ps3 .> P2
.. |
.. | psX -> P1
.. | psX .left.> P2
.. |
.. | ps1 -[hidden]down- ps2
.. | ps2 -[hidden]right- psX
.. | ps3 -[hidden]down- psX
.. |
.. | @enduml

Note that several *servers* can share the same *IP address* but their port numbers should be different. Likewise,
several *servers* can share the same port if their *IP addresses* are different.

+--------------------------------------------------------+--------------------------------------------------------+
| **SERVER**                                             | **CLIENT**                                             |
+--------------------------------------------------------+--------------------------------------------------------+
| **C++**                                                | **C++**                                                |
+--------------------------------------------------------+--------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp             | .. literalinclude:: ../code/CodeTester.cpp             |
|    :language: c++                                      |    :language: c++                                      |
|    :start-after: //CONF_DS_REDUNDANCY_SCENARIO_SERVER  |    :start-after: //CONF_DS_REDUNDANCY_SCENARIO_CLIENT  |
|    :end-before: //!--                                  |    :end-before: //!--                                  |
+--------------------------------------------------------+--------------------------------------------------------+
| **XML**                                                | **XML**                                                |
+--------------------------------------------------------+--------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml              | .. literalinclude:: ../code/XMLTester.xml              |
|    :language: xml                                      |    :language: xml                                      |
|    :start-after: <!-->CONF_DS_RDNCY_SCENARIO_SERVER<-->|    :start-after: <!-->CONF_DS_RDNCY_SCENARIO_CLIENT<-->|
|    :end-before: <!--><-->                              |    :end-before: <!--><-->                              |
+--------------------------------------------------------+--------------------------------------------------------+

.. _discovery_server_persistency_scenario_setup:

UDPv4 persistency example
^^^^^^^^^^^^^^^^^^^^^^^^^

All participants keeps record of all endpoints discovered (other participants, subscribers or publishers). Different
kind of participants populate this record with different procedures:

- *clients* receive this information from its *servers*.
- *servers* receive this information from its *clients*.

Given that *servers* used to have many *clients* associated, this is a lengthy process. In case of *server* failure we
may be interested in speed up this process when the *server* restarts.

Keep the discovery information in a file synchronize with the *server*'s record fulfills the goal. In order to enable
this we must just specify the :ref:`discovery protocol <discovery_protocol>` as **BACKUP**.

Once the *server* is created it generates a *server-<GUIDPREFIX>.db* (*exempli gratia
server-73-65-72-76-65-72-63-6C-69-65-6E-74.db*) on its process working directory.

In order to start afresh, that is without deserialize any discovery info, the old backup file must be removed or renamed
before launching the server.

.. _discovery_server_partitioning_setup:

UDPv4 partitioning using servers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Server* association can be seen as another isolation mechanism besides :ref:`domains <listening_locators>` and
:ref:`partitions <partitions>`. *Clients* that do not share a *server* cannot see each other and belong to isolated
server networks. In order to connect server isolated networks we can:

    1. Connect each *client* to both *servers*.
    2. Connect one *server* to the other.
    3. Create a new *server* linked to the *servers* to which the clients are connected.

Options 1 and 2 can only be implemented by modifying attributes or XML configuration files beforehand. In this regard
they match the domain and partition strategy. Option 3 can be implemented at runtime, that is, when the isolated
networks are already up and running.

.. image:: ds_partition.png
    :align: center
    :width: 75%

.. | @startuml
.. |
.. | package "Option 1 | Static" {
.. |
.. | component [Server 1] as 1_s1
.. | component [Server 2] as 1_s2
.. | (client 1) as 1_c1
.. | (client 2) as 1_c2
.. |
.. | 1_s2 -[hidden]up- 1_s1
.. | 1_c2 -[hidden]up- 1_c1
.. |
.. | }
.. |
.. | 1_s1 <- 1_c1
.. | 1_s2 <- 1_c2
.. |
.. | 1_s1 <- 1_c2
.. | 1_s2 <-left- 1_c1
.. |
.. | package "Option 2 | Static" {
.. |
.. | component [Server 1] as 2_s1
.. | component [Server 2] as 2_s2
.. | (client 1) as 2_c1
.. | (client 2) as 2_c2
.. |
.. | 2_s2 -up- 2_s1
.. | 2_c2 -[hidden]up- 2_c1
.. |
.. | }
.. |
.. | 2_s1 <- 2_c1
.. |
.. | 2_s2 <- 2_c2
.. |
.. | package "Option 3 | Dynamic" {
.. |
.. | component [Server 1] as 3_s1
.. | component [Server 2] as 3_s2
.. | component [Aux Server] as aux
.. |
.. | (client 1) as 3_c1
.. | (client 2) as 3_c2
.. |
.. | 3_s2 <-up- aux
.. | aux -up-> 3_s1
.. | 3_c2 -[hidden]up- aux
.. | aux -[hidden]up- 3_c1
.. | }
.. |
.. | 3_s1 <-right- 3_c1
.. |
.. | 3_s2 <-right- 3_c2
.. |
.. | @enduml

Option 1
""""""""

Connect each *client* to both *servers*. This case matches the :ref:`redundancy use case
<discovery_server_redundancy_scenario_setup>` already introduced.

Option 2
""""""""

Connect one *server* to the other. In this case we consider two servers, each one managing an isolated network:

.. csv-table::
    :header: "Network", "Prefix", "UDPv4 address"
    :widths: 4,20,100

    A, 75.63.2D.73.76.72.63.6C.6E.74.2D.31, "192.168.10.60:56543"
    B, 75.63.2D.73.76.72.63.6C.6E.74.2D.32, "192.168.10.57:56542"

In order to communicate both networks we can setup server A to act as client of server B as follows:

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp             |
|    :language: c++                                      |
|    :start-after: //CONF_DS_PARTITION_2                 |
|    :end-before: //!--                                  |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml              |
|    :language: xml                                      |
|    :start-after: <!-->CONF_DS_PARTITION_2<-->          |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

Option 3
""""""""

Create a new *server* linked to the *servers* to which the clients are connected. In this case we have two isolated
networks A and B, which may be up and running, and join them with a server C.

.. csv-table::
    :header: "Server", "Prefix", "UDPv4 address"
    :widths: 4,20,100

    A, 75.63.2D.73.76.72.63.6C.6E.74.2D.31, "192.168.10.60:56543"
    B, 75.63.2D.73.76.72.63.6C.6E.74.2D.32, "192.168.10.57:56542"
    C, 75.63.2D.73.76.72.63.6C.6E.74.2D.33, "192.168.10.54:56541"

In order to communicate both networks we can setup server C to act as client of servers A and B as follows:

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp             |
|    :language: c++                                      |
|    :start-after: //CONF_DS_PARTITION_3                 |
|    :end-before: //!--                                  |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml              |
|    :language: xml                                      |
|    :start-after: <!-->CONF_DS_PARTITION_3<-->          |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

.. _wide_deployments_static:

Well Known Network Topologies
-----------------------------

It is often the case in industrial deployments, such as productions lines, that the entire network topology (hosts, IP
addresses, etc.) is known beforehand.
Such scenarios are perfect candidates for Fast-RTPS STATIC discovery mechanism, which drastically reduces the middleware
setup time (time until all the entities are ready for information exchange), while at the same time limits the
connections to those strictly necessary.
As explained in the :ref:`discovery` section, all Fast-RTPS discovery mechanisms consist of two steps: PDP and EDP.

.. _wide_deployments_static_pdp:

Peer-to-Peer Participant Discovery Phase
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, Fast-RTPS uses SPDP protocol for the PDP phase.
This entails the participants sending periodic PDP announcements over a well known multicast addresses, using IP ports
calculated from the domain.
For large deployments, this can result in quite some meta traffic, since whenever a participant receives a PDP message
via multicast, it replies to the remote participant using an address and port specified in the message.
In this scenario the number of PDP connections is *N * (N - 1)*, with *N* being the number of participants in the
network.

However, it is often the case that not all the participants need to be aware of all the rest of the remote participants
present in the network.
For limiting all this PDP meta traffic, Fast-RTPS participants can be configured to send their PDP announcements only to
the remote participants to which they are required to connect.
This is done by specifying a list of peers as a set of IP address-port pairs, and by disabling the participant multicast
announcements.
Use-case :ref:`use-case-fast-rtps-over-wifi` provides a detailed explanation on how to configure Fast-RTPS for such
case.

.. _wide_deployments_static_edp:

STATIC Endpoint Discovery Phase
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As explained in :ref:`discovery_static`, the EDP meta traffic can be completely avoided by specifying the EDP discovery
using XML files.
This way, the user can manually configure which publisher/subscriber matches with which one, so they can start sharing
user data right away.
To do that, a STATIC discovery XML file must be supplied to the local entity describing the configuration of the remote
entity.
In this example, a publisher in topic ``HelloWorldTopic`` from participant ``HelloWorldPublisher`` is matched with a
subscriber from participant ``HelloWorldSubscriber``.
A fully functional example implementing STATIC EDP is
`STATIC EDP example <https://github.com/eProsima/Fast-RTPS/blob/master/examples/C%2B%2B/StaticHelloWorldExample>`_.

Create STATIC discovery XML files
"""""""""""""""""""""""""""""""""

   +-----------------------------------------------------+-----------------------------------------------------+
   | **HelloWorldPublisher.xml**                         | **HelloWorldSubscriber.xml**                        |
   +-----------------------------------------------------+-----------------------------------------------------+
   | .. literalinclude:: ../code/StaticTester.xml        | .. literalinclude:: ../code/StaticTester.xml        |
   |    :language: xml                                   |    :language: xml                                   |
   |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_PUB |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_SUB |
   |    :end-before: <!--><-->                           |    :end-before: <!--><-->                           |
   +-----------------------------------------------------+-----------------------------------------------------+

Create entities and load STATIC discovery XML files
"""""""""""""""""""""""""""""""""""""""""""""""""""

When creating the entities, the local publisher/subscriber attributes must match those defined in the STATIC discovery
XML file loaded by the remote entity.

   +-----------------------------------------------------+-----------------------------------------------------+
   | **PUBLISHER**                                       | **SUBSCRIBER**                                      |
   +-----------------------------------------------------+-----------------------------------------------------+
   | **C++**                                             | **C++**                                             |
   +-----------------------------------------------------+-----------------------------------------------------+
   | .. literalinclude:: ../code/CodeTester.cpp          | .. literalinclude:: ../code/CodeTester.cpp          |
   |    :language: c++                                   |    :language: c++                                   |
   |    :start-after: //STATIC_DISCOVERY_USE_CASE_PUB    |    :start-after: //STATIC_DISCOVERY_USE_CASE_SUB    |
   |    :end-before: //!--                               |    :end-before: //!--                               |
   +-----------------------------------------------------+-----------------------------------------------------+
   | **XML**                                             | **XML**                                             |
   +-----------------------------------------------------+-----------------------------------------------------+
   | .. literalinclude:: ../code/XMLTester.xml           | .. literalinclude:: ../code/XMLTester.xml           |
   |    :language: xml                                   |    :language: xml                                   |
   |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_PUB |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_SUB |
   |    :end-before: <!--><-->                           |    :end-before: <!--><-->                           |
   +-----------------------------------------------------+-----------------------------------------------------+

.. _fastrtps_ros2:

Fast-RTPS in ROS 2
==================

Fast-RTPS is the default middleware implementation in the
`Open Source Robotic Fundation (OSRF) <https://www.openrobotics.org/>`_
`Robot Operating System ROS 2 <https://index.ros.org/doc/ros2/>`_.
This tutorial is an explanation of how to take full advantage of Fast-RTPS wide set of capabilities in a ROS 2 project.

The interface between the ROS2 stack and Fast-RTPS is provided by a ROS 2 package
`rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`_.
This package is available in all ROS 2 distributions, both from binaries and from sources.
``rmw_fastrtps`` actually provides not one but two different ROS 2 middleware implementations, both of them using
Fast-RTPS as middleware layer: ``rmw_fastrtps_cpp`` and ``rmw_fastrtps_dynamic_cpp``.
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

Configuring Fast-RTPS with XML files
-------------------------------------

As described in :ref:`xml-profiles` section, there are two possibilities for providing Fast-RTPS with XML configuration
files:

* **Recommended**: Define the location of the XML configuration file with environment variable
  ``FASTRTPS_DEFAULT_PROFILES_FILE``.

  ::

      export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>

* **Alternative**: Create a *DEFAULT_FASTRTPS_PROFILES.xml* and place it in the same directory as the application
  executable.

Default profiles
^^^^^^^^^^^^^^^^

Under ROS 2, the entity creation does not allow for selecting different profiles from the XML.
To work around this issue, the profiles can be marked with an attribute ``is_default_profile="true"``, so when an entity
of that type is created, it will automatically load that profile.
The mapping between ROS 2 entities and Fast-RTPS entities is:

+--------------+------------------------+
| ROS entity   | Fast-RTPS entity       |
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
| .. literalinclude:: ../code/XMLTester.xml               |
|    :language: xml                                       |
|    :start-after: <!-->CONF_ROS2_DEFAULT_PROFILE         |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

Configure Publication Mode and History Memory Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, ``rmw_fastrtps`` sets some of the Fast-RTPS configurable parameters, ignoring whatever configuration is
provided in the XML file.
Said parameters, and their default values under ROS 2, are:

+-----------------------+--------------------------------------------------+-------------------------------------------+
| Parameter             | Description                                      | Default ROS 2 value                       |
+=======================+==================================================+===========================================+
| History memory policy | Fast-RTPS preallocates memory for the publisher  | ``PREALLOCATED_WITH_REALLOC_MEMORY_MODE`` |
|                       | and subscriber histories.                        |                                           |
|                       | When those histories fill up, a reallocation     |                                           |
|                       | occurs to reserve more memory.                   |                                           |
+-----------------------+--------------------------------------------------+-------------------------------------------+
| Publication mode      | User calls to publication method add the         | ``ASYNCHRONOUS_PUBLISH_MODE``             |
|                       | messages in a queue that is managed in a         |                                           |
|                       | different thread, meaning that the user thread   |                                           |
|                       | is available right after the call to send data.  |                                           |
+-----------------------+--------------------------------------------------+-------------------------------------------+

However, it is possible to fully configure Fast-RTPS (including the history memory policy and the publication mode)
using an XML file in combination with environment variable ``RMW_FASTRTPS_USE_QOS_FROM_XML``.

::

    export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_xml_file>
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
    ros2 run <package> <application>

.. _ros2_example:

Example
-------

The following example uses the ROS 2 talker/listener demo, configuring Fast-RTPS to publish synchronously, and to have a
dynamically allocated publisher and subscriber histories.

#. Create a XML file `ros_example.xml` and save it in `path/to/xml/`

   +---------------------------------------------------------+
   | **XML**                                                 |
   +---------------------------------------------------------+
   | .. literalinclude:: ../code/XMLTester.xml               |
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
