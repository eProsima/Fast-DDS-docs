Typical Use-Cases
#################

.. START Introduction

.. END Introduction



.. START SEC:FAST-RTPS-OVER-WIFI

.. _use-case-fast-rtps-over-wifi:

Fast-RTPS over WIFI
===================

The `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ defines the SIMPLE discovery as the default
mechanism for discovering participants in the network.
One of the main features of this mechanism is the use of multicast communication in the Participant Discovery
Phase (PDP). This could be a problem in case the communication is not wired, i.e. WiFi communication. Fast-RTPS'
solution to this challenge is to define the participants with which a unicast communication is to be set, i.e an
initial list of remote peers.

.. _use-case-initial-peers:

Initial Peers
-------------

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), each participant must
listen for incoming
PDP discovery metatraffic in two different ports, one linked with a multicast address, and another one linked to a
unicast address.
Fast-RTPS allows for the configuration of an initial peers list which contains one or much such address-port pairs
corresponding to remote participants PDP discovery listening resources, so that the local participant will not only
sent its PDP traffic to the default multicast address-port specified by its domain, but also to all the address-port
pairs specified in the :ref:`initial-peers` list.

A participant's initial peers list contains the list of address-port pairs of all other participants with
which it will communicate. It is a list of addresses that a participant will use in the unicast discovery mechanism,
together or as an alternative to multicast discovery. Therefore, this approach also applies to those scenarios in which
multicast functionality is not available.

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), the participants'
discovery traffic
unicast listening ports are calculated using the following equation:
7400 + 250 * `domainID` + 10 + 2 * `participantID`. Thus, if a participant operates in Domain 0 (default domain) and
its ID is 1, its discovery traffic unicast listening port would be: 7400 + 250 * 0 + 10 + 2 * 1 = 7412.

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

If all the peers are known beforehand, it is possible to disable the multicast meta traffic completely. This is done
using the configuration attribute ``metatrafficUnicastLocatorList``. By defining a custom
``metatrafficUnicastLocatorList``, the default metatraffic multicast and unicast locators to be employed by the
participant are avoided, which prevents the participant from listening to any discovery data from multicast sources.
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

.. END SEC:FAST-RTPS-OVER-WIFI



.. START SEC:FAST-RTPS-WIDE-DEPLOYMENTS

.. _wide_deployments:

Wide Deployments
================

.. START SUBSEC:DISCOVERY-SERVER

Systems with large amounts of communication nodes might pose a challenge to
`Data Distribution Service (DDS) <https://www.omg.org/spec/DDS/1.4/PDF>`_ based middleware implementations in terms of
setup times, memory consumption, and network load. This is because, as explained in :ref:`discovery`, the Participant
Discovery Phase (PDP) relies on meta traffic announcements send to multicast addresses so that all the participants in
the network can acknowledge each other. This phase is followed by a Endpoint Discovery Phase (EDP) where all the
participants exchange information (using unicast addresses) about their publisher and subscriber entities with the rest
of the participants, so that matching between publishers and subscribers in the same topic can occur. As the number of
participants, publishers, and subscribers increases, the meta-traffic, as well as the number of connections, increases
exponentially, severely affecting the setup time and memory consumption. Fast-RTPS provides extra features that expand
the DDS standard to adapt it to wide deployment scenarios.

+-----------------------------------+---------------------------------------------------------------------------------+
| Feature                           | Purpose                                                                         |
+===================================+=================================================================================+
| Server-Client Discovery Mechanism | This feature is intended to substitute the standard SEDP and SPDP protocol with |
|                                   | a discovery based on a server-client architecture, where all the meta-traffic   |
|                                   | goes through a hub (server) to be distributed throughout the network            |
|                                   | communication nodes.                                                            |
+-----------------------------------+---------------------------------------------------------------------------------+
| Static Discovery                  | With this feature, the user can manually specify which participant should       |
|                                   | communicate with which one and through which address and port. Furthermore, the |
|                                   | the user can specify which publisher/subscriber matches with which one, thus    |
|                                   | eliminating all EDP meta traffic.                                               |
+-----------------------------------+---------------------------------------------------------------------------------+

.. END SUBSEC:DISCOVERY-SERVER


.. START SUBSEC:STATIC-DISCOVERY

.. _wide_deployments_static:

Well Known Network Topologies
-----------------------------

It is often the case in industrial deployments, such as productions lines, that the entire network topology (hosts, IP
addresses, etc.) is known beforehand. Such scenarios are perfect candidates for Fast-RTPS STATIC discovery mechanism,
which drastically reduces the middleware setup time (time until all the entities are ready for information exchange),
while at the same time limits the connections to those strictly necessary. As explained in the :ref:`discovery` section,
all Fast-RTPS discovery mechanisms consist of two steps: Participant Discovery Phase (PDP), and Endpoint Discovery Phase
(EDP).

.. _wide_deployments_static_pdp:

Peer-to-Peer Participant Discovery Phase
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, Fast-RTPS uses Simple Participant Discovery Protocol (SPDP) for PDP phase. This entails the participants
sending periodic PDP announcements over a well known multicast addresses, using IP ports calculated from the domain. For
large deployments, this can result in quite some meta traffic, since whenever a participant receives a PDP message via
multicast, it replays to the remote participant using an address and port specified in the message. In this scenario the
number of PDP connections is *N * (N - 1)*, with *N* being the number of participants in the network.

However, it is often the case that not all the participants need to be aware of all the rest of the remote participants
present in the network. For limiting all this PDP meta traffic, Fast-RTPS participants can be configured to send their
PDP announcements only to the remote participants to which they are required to connect. This is done by specifying a
list of peers as a set of IP address-port pairs, and by disabling the participant multicast announcements. Use-case
:ref:`use-case-fast-rtps-over-wifi` provides a detailed explanation on how to configure Fast-RTPS for such case.

.. _wide_deployments_static_edp:

STATIC Endpoint Discovery Phase
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As explained in :ref:`discovery_static`, the EDP meta traffic can be completely avoided by specifying the EDP discovery
using XML files. This way, the user can manually configure which publisher/subscriber matches with which one, so they
can start sharing user data right away. To do that, a STATIC discovery XML file must be supplied to the local entity
describing the configuration of the remote entity. In this example, a publisher in topic ``HelloWorldTopic`` from
participant ``HelloWorldPublisher`` is matched with a subscriber from participant ``HelloWorldSubscriber``. A fully
functional example implementing STATIC EDP is
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

.. END SUBSEC:STATIC-DISCOVERY


.. END SEC:FAST-RTPS-WIDE-DEPLOYMENTS



.. START SEC:FAST-RTPS-IN-ROS2

.. _fastrtps_ros2:

Fast-RTPS in ROS 2
==================

Fast-RTPS is the default middleware implementation in the `Open Source Robotic Fundation (OSRF) <https://www.openrobotics.org/>`_
`Robot Operating System ROS 2 <https://index.ros.org/doc/ros2/>`_. This tutorial is an explanation of how to take full
advantage for Fast-RTPS wide set of capabilities in a ROS 2 project.

The interface between the ROS2 stack and Fast-RTPS is provided by a ROS 2 package
`rmw_fastrtps <https://raw.githubusercontent.com/ros2/rmw_fastrtps/>`_. This packages is available in all ROS 2
distributions, both from binaries and from sources. ``rmw_fastrtps`` actually provides not one but two different ROS 2
middleware implementations, both of them using Fast-RTPS as middleware layer: ``rmw_fastrtps_cpp`` and
``rmw_fastrtps_dynamic_cpp``. The main difference between the two is that ``rmw_fastrtps_dynamic_cpp`` uses
introspection type support at run time to decide on the serialization/deserialization mechanism, while
``rmw_fastrtps_cpp`` uses its own type support, which generates the mapping for each message type at build time. The
default ROS 2 RMW implementation is ``rmw_fastrtps_cpp``. However, it is still possible to select
``rmw_fastrtps_dynamic_cpp`` using the environment variable ``RMW_IMPLEMENTATION``:

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

Under ROS 2, the entity creation does not allow for selecting different profiles from the XML. To work around this
issue, the profiles can be marked with an attribute ``is_default_profile="true"``, so when an entity of that type is
created, it will automatically load that profile. The mapping between ROS 2 entities and Fast-RTPS entities is:

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
provided in the XML file. Said parameters, and their default values under ROS 2, are:

+-----------------------+--------------------------------------------------+-------------------------------------------+
| Parameter             | Description                                      | Default ROS 2 value                       |
+=======================+==================================================+===========================================+
| History memory policy | Fast-RTPS preallocates memory for the publisher  | ``PREALLOCATED_WITH_REALLOC_MEMORY_MODE`` |
|                       | and subscriber histories. When those histories   |                                           |
|                       | fill up, a reallocation occurs to reserve more   |                                           |
|                       | memory                                           |                                           |
+-----------------------+--------------------------------------------------+-------------------------------------------+
| Publication mode      | User calls to publication method add the         | ``ASYNCHRONOUS_PUBLISH_MODE``             |
|                       | messages in a queue that is managed in a         |                                           |
|                       | different thread, meaning that the user thread   |                                           |
|                       | is available right after the call to send data   |                                           |
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

.. END SEC:FAST-RTPS-IN-ROS2
