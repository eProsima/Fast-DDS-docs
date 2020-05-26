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
| .. literalinclude:: /../code/CodeTester.cpp            | .. literalinclude:: /../code/CodeTester.cpp            |
|    :language: c++                                      |    :language: c++                                      |
|    :start-after: //CONF_DS_MAIN_SCENARIO_SERVER        |    :start-after: //CONF_DS_MAIN_SCENARIO_CLIENT        |
|    :end-before: //!--                                  |    :end-before: //!--                                  |
+--------------------------------------------------------+--------------------------------------------------------+
| **XML**                                                | **XML**                                                |
+--------------------------------------------------------+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             | .. literalinclude:: /../code/XMLTester.xml             |
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

.. image:: /01-figures/ds_redundancy.png
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
| .. literalinclude:: /../code/CodeTester.cpp            | .. literalinclude:: /../code/CodeTester.cpp            |
|    :language: c++                                      |    :language: c++                                      |
|    :start-after: //CONF_DS_REDUNDANCY_SCENARIO_SERVER  |    :start-after: //CONF_DS_REDUNDANCY_SCENARIO_CLIENT  |
|    :end-before: //!--                                  |    :end-before: //!--                                  |
+--------------------------------------------------------+--------------------------------------------------------+
| **XML**                                                | **XML**                                                |
+--------------------------------------------------------+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             | .. literalinclude:: /../code/XMLTester.xml             |
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

.. image:: /01-figures/ds_partition.png
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
| .. literalinclude:: /../code/CodeTester.cpp            |
|    :language: c++                                      |
|    :start-after: //CONF_DS_PARTITION_2                 |
|    :end-before: //!--                                  |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
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
| .. literalinclude:: /../code/CodeTester.cpp            |
|    :language: c++                                      |
|    :start-after: //CONF_DS_PARTITION_3                 |
|    :end-before: //!--                                  |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
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
   | .. literalinclude:: /../code/StaticTester.xml       | .. literalinclude:: /../code/StaticTester.xml       |
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
   | .. literalinclude:: /../code/CodeTester.cpp         | .. literalinclude:: /../code/CodeTester.cpp         |
   |    :language: c++                                   |    :language: c++                                   |
   |    :start-after: //STATIC_DISCOVERY_USE_CASE_PUB    |    :start-after: //STATIC_DISCOVERY_USE_CASE_SUB    |
   |    :end-before: //!--                               |    :end-before: //!--                               |
   +-----------------------------------------------------+-----------------------------------------------------+
   | **XML**                                             | **XML**                                             |
   +-----------------------------------------------------+-----------------------------------------------------+
   | .. literalinclude:: /../code/XMLTester.xml          | .. literalinclude:: /../code/XMLTester.xml          |
   |    :language: xml                                   |    :language: xml                                   |
   |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_PUB |    :start-after: <!-->STATIC_DISCOVERY_USE_CASE_SUB |
   |    :end-before: <!--><-->                           |    :end-before: <!--><-->                           |
   +-----------------------------------------------------+-----------------------------------------------------+

