Typical Use-Cases
#################

.. START Introduction

.. END Introduction



.. START SEC:FAST-RTPS-OVER-WIFI


.. START SUBSEC:INITIAL-PEERS

Initial Peers Use Case
======================

This section defines the use-case resulting from the application of the :ref:`initial-peers` functionality provided
by Fast-RTPS.

An initial peers list is
created for each participant, in which the unicast locators of the other participants with which it will communicate
are defined, that is, a list of addresses is defined to be used in the discovery mechanism that is used together or
as an alternative to multicast discovery. Therefore, this applies to those scenarios in which multicast
functionality is not available.

The definition of the unicast ports of each participants is made according to the following equation:
7400 + 250 * `domainID` + 10 + 2 * `participantID`. Thus, if a participant operates in Domain 0 (default domain) and
its ID is 1, its port would be: 7400 + 250 * 0 + 10 + 2 * 1 = 7412.

An example of the implementation of Initial Peers in the participant profile is presented below.

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

.. END SUBSEC:INITIAL-PEERS


.. START SUBSEC:DISABLE-MULTICAST

Disabling multicast discovery
-----------------------------

Another possible use case is the definition of initial peers for the participant to discover these only. This is done
using the configuration attribute `metatrafficUnicastLocatorList`. By defining a custom
`metatrafficUnicastLocatorList`, the default metatraffic multicast and unicast locators to be employed by the
participant is prevented; this prevents the participant from listening any discovery data from multicast.

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



.. END SUBSEC:DISABLE-MULTICAST


.. END SEC:FAST-RTPS-OVER-WIFI



.. START SEC:FAST-RTPS-WIDE-DEPLOYMENTS


.. START SUBSEC:DISCOVERY-SERVER

.. END SUBSEC:DISCOVERY-SERVER


.. START SUBSEC:STATIC-DISCOVERY

.. END SUBSEC:STATIC-DISCOVERY


.. END SEC:FAST-RTPS-WIDE-DEPLOYMENTS



.. START SEC:FAST-RTPS-IN-ROS2

.. END SEC:FAST-RTPS-IN-ROS2
