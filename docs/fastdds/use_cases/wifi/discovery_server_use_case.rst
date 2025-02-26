.. _discovery-server-use-case:

Discovery Server
=================

During :ref:`discovery`, the Participant Discovery Phase (PDP) relies on meta traffic
announcements sent to multicast addresses so that all the :ref:`DomainParticipants<dds_layer_domainParticipant>`
in the network can acknowledge each other.
This phase is followed by a Endpoint Discovery Phase (EDP) where all the
DomainParticipants use discovered unicast addresses to exchange information about
their :ref:`dds_layer_publisher` and :ref:`dds_layer_subscriber` entities with the rest of the
DomainParticipants, so that matching between entities of the same topic can occur.

*Fast DDS* provides a client-server discovery mechanism, in which a server DomainParticipant operates
as the central point of communication.
It collects and processes the metatraffic sent by the client DomainParticipants,
and then distributes the appropriate information among the rest of the clients.

A complete description of the feature can be found at :ref:`discovery_server`.
The following subsections present configurations for different discovery server use cases.

.. _discovery_server_major_scenario_setup:

UDPv4 basic example setup
-------------------------

To configure the Discovery Server scenario, two types of participants are created: the server participant and
the client participant.
Only one parameter needs be configured in this type of implementation:

+ **Server Address-port pair**: Specifies the IP address and port of the machine that implements the server.
  Any free random port can be used.
  However, using :ref:`RTPS standard ports<listening_locators_defaultPorts>` is discouraged.

+-------------------------------------------------------------+
| **SERVER**                                                  |
+=============================================================+
|                                                             |
|.. tab-set::                                                 |
|                                                             |
|   .. tab-item:: C++                                         | 
|      :sync: cpp                                             |
|                                                             |
|      .. literalinclude:: /../code/DDSCodeTester.cpp         |
|         :language: c++                                      |
|         :start-after: //CONF_DS_MAIN_SCENARIO_SERVER        |
|         :end-before: //!--                                  |
|         :dedent: 8                                          |
|                                                             |
|   .. tab-item:: XML                                         |
|      :sync: xml                                             |
|                                                             |
|      .. literalinclude:: /../code/XMLTester.xml             |
|         :language: xml                                      |
|         :start-after: <!-->CONF_DS_MAIN_SCENARIO_SERVER<--> |
|         :end-before: <!--><-->                              |
|         :lines: 2-3,5-                                      |
|         :append: </profiles>                                |
+-------------------------------------------------------------+

+-------------------------------------------------------------+
| **CLIENT**                                                  |
+=============================================================+
|                                                             |
|.. tab-set::                                                 |
|                                                             |
|   .. tab-item:: C++                                         | 
|      :sync: cpp                                             |
|                                                             |
|      .. literalinclude:: /../code/DDSCodeTester.cpp         |
|         :language: c++                                      |
|         :start-after: //CONF_DS_MAIN_SCENARIO_CLIENT        |
|         :end-before: //!--                                  |
|         :dedent: 8                                          |
|                                                             |
|   .. tab-item:: XML                                         |
|      :sync: xml                                             |
|                                                             |
|      .. literalinclude:: /../code/XMLTester.xml             |
|         :language: xml                                      |
|         :start-after: <!-->CONF_DS_MAIN_SCENARIO_CLIENT<--> |
|         :end-before: <!--><-->                              |
|         :lines: 2-3,5-                                      |
|         :append: </profiles>                                |
+-------------------------------------------------------------+

.. _discovery_server_redundancy_scenario_setup:

UDPv4 redundancy example
------------------------

The :ref:`basic setup example<discovery_server_major_scenario_setup>` presents a *single point of failure*.
That is, if the server fails the clients are not able to perform the discovery.
To prevent this, several servers could be linked to each client.
Then, a discovery failure only takes place if *all servers* fail, which is a more unlikely event.

In the example below, the values have been chosen to ensure each server has a unique
*unicast address-port pair*.
Note that several servers can share the same IP address but their port numbers should be different.
Likewise, several servers can share the same port if their IP addresses are different.


.. csv-table::
    :header: "Prefix", "UDPv4 address-port"
    :widths: 20,100

    75.63.2D.73.76.72.63.6C.6E.74.2D.31, "192.168.10.57:56542"
    75.63.2D.73.76.72.63.6C.6E.74.2D.32, "192.168.10.60:56543"

.. image:: /01-figures/ds_redundancy.svg
    :align: center

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


+-------------------------------------------------------------+
| **SERVER**                                                  |
+=============================================================+
|                                                             |
| .. tab-set::                                                |
|                                                             |
|   .. tab-item:: C++                                         |
|      :sync: cpp                                             |
|                                                             |
|      .. literalinclude:: /../code/DDSCodeTester.cpp         |
|         :language: c++                                      |
|         :start-after: //CONF_DS_REDUNDANCY_SCENARIO_SERVER  |
|         :end-before: //!--                                  |
|         :dedent: 8                                          |
|                                                             |
|   .. tab-item:: XML                                         |
|      :sync: xml                                             |
|                                                             |
|      .. literalinclude:: /../code/XMLTester.xml             |
|         :language: xml                                      |
|         :start-after: <!-->CONF_DS_RDNCY_SCENARIO_SERVER<-->|
|         :end-before: <!--><-->                              |
|         :lines: 2-3,5-                                      |
|         :append: </profiles>                                |
+-------------------------------------------------------------+

+-------------------------------------------------------------+
| **CLIENT**                                                  |
+=============================================================+
|                                                             |
| .. tab-set::                                                |
|                                                             |
|   .. tab-item:: C++                                         |
|      :sync: cpp                                             |
|                                                             |
|      .. literalinclude:: /../code/DDSCodeTester.cpp         |
|         :language: c++                                      |
|         :start-after: //CONF_DS_REDUNDANCY_SCENARIO_CLIENT  |
|         :end-before: //!--                                  |
|         :dedent: 8                                          |
|                                                             |
|   .. tab-item:: XML                                         |
|      :sync: xml                                             |
|                                                             |
|      .. literalinclude:: /../code/XMLTester.xml             |
|         :language: xml                                      |
|         :start-after: <!-->CONF_DS_RDNCY_SCENARIO_CLIENT<-->|
|         :end-before: <!--><-->                              |
|         :lines: 2-3,5-                                      |
|         :append: </profiles>                                |
+-------------------------------------------------------------+

.. _discovery_server_persistency_scenario_setup:

UDPv4 persistency example
-------------------------

On Discovery Server, servers gather and maintain the information of all connected endpoints,
and distribute it to the clients.
In case of a server failure, all this information is lost and the server needs to recover it on restart.
In the :ref:`basic setup<discovery_server_major_scenario_setup>` this is done
starting over the :ref:`discovery` process.
Given that servers usually have lots of clients associated, this is very time consuming.

Alternatively, *Fast DDS* allows to synchronize the server's discovery record to a file, so that the information can be
loaded back into memory during the restart.
This feature is enabled specifying the :ref:`discovery_protocol` as **BACKUP**.

The record file is located on the server's process working directory, and named following the pattern
*server-<GUIDPREFIX>.db* (for example: *server-73-65-72-76-65-72-63-6C-69-65-6E-74.db*).
Once the server is created, it automatically looks for this file.
If it already exists, its contents are loaded, avoiding the need of re-discovering the clients.
To make a fresh restart, any such backup file must be removed or renamed before launching the server.


.. _discovery_server_partitioning_setup:

UDPv4 partitioning using servers
--------------------------------

Server association can be seen as another isolation mechanism besides :ref:`Domains <dds_layer_domain>` and
:ref:`partitions`.
Clients that do not share a server cannot see each other and belong to isolated server networks.
For example, in the following figure, *client 1* and *client 2* cannot communicate even if they are on the
same physical network and Domain.

.. figure:: /01-figures/ds_partition.svg
    :align: center

    Clients cannot see each other due to server isolation

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
.. | @enduml

However, it is possible to connect server isolated networks very much as physical networks
can be connected through routers:

    * :ref:`discovery_server_partitioning_option1`:
      Connecting the clients to several servers, so that the clients belong to several networks.
    * :ref:`discovery_server_partitioning_option2`:
      Connecting one server to another, so that the networks are linked together.
    * :ref:`discovery_server_partitioning_option3`:
      Create a new server linked to the servers to which the clients are connected.

Options 1 and 2 can only be implemented by modifying QoS values or XML configuration files beforehand.
In this regard they match the domain and partition strategy.
Option 3, however, can be implemented at runtime, when the isolated networks are already up and running.

.. image:: /01-figures/ds_partition_link.svg
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

.. _discovery_server_partitioning_option1:

Option 1
^^^^^^^^

Connect each client to both servers.
This case matches the :ref:`redundancy use case <discovery_server_redundancy_scenario_setup>` already introduced.

.. _discovery_server_partitioning_option2:

Option 2
^^^^^^^^

Connect one server to the other.
This means configuring one of the servers to act as a client of the other.

Consider two servers, each one managing an isolated network:

.. csv-table::
    :header: "Network", "Prefix", "UDPv4 address"
    :widths: 4,20,100

    A, 75.63.2D.73.76.72.63.6C.6E.74.2D.31, "192.168.10.60:56543"
    B, 75.63.2D.73.76.72.63.6C.6E.74.2D.32, "192.168.10.57:56542"

In order to communicate both networks we can set server A to act as  a client of server B:

.. tab-set::

   .. tab-item:: **C++**
      :sync: cpp

      .. literalinclude:: /../code/DDSCodeTester.cpp
         :language: c++
         :start-after: //CONF_DS_PARTITION_2
         :end-before: //!--
         :dedent: 8

   .. tab-item:: **XML**
      :sync: xml

      .. literalinclude:: /../code/XMLTester.xml
         :language: xml
         :start-after: <!-->CONF_DS_PARTITION_2<-->
         :end-before: <!--><-->
         :lines: 2-3,5-
         :append: </profiles>

.. _discovery_server_partitioning_option3:

Option 3
^^^^^^^^

Create a new server linked to the servers to which the clients are connected.

Consider two servers (A and B), each one managing an isolated network, and a third
server (C) that will be used to connect the first two:

.. csv-table::
    :header: "Server", "Prefix", "UDPv4 address"
    :widths: 4,20,100

    A, 75.63.2D.73.76.72.63.6C.6E.74.2D.31, "192.168.10.60:56543"
    B, 75.63.2D.73.76.72.63.6C.6E.74.2D.32, "192.168.10.57:56542"
    C, 75.63.2D.73.76.72.63.6C.6E.74.2D.33, "192.168.10.54:56541"

In order to communicate both networks we can setup server C to act as client of servers A and B as follows:

.. tab-set::

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../code/DDSCodeTester.cpp
         :language: c++
         :start-after: //CONF_DS_PARTITION_3
         :end-before: //!--
         :dedent: 8

   .. tab-item:: XML
      :sync: xml

      .. literalinclude:: /../code/XMLTester.xml
         :language: xml
         :start-after: <!-->CONF_DS_PARTITION_3<-->
         :end-before: <!--><-->
         :lines: 2-3,5-
         :append: </profiles>

.. note::
     GUID Prefixes are used in these examples to identify the *servers* and help to understand each scenario.
     However, they are not mandatory and can be omitted.
     Note that in the *clients* configuration, the GUID Prefix is always missing, as it is not needed in
     order to connect to the *servers*.
