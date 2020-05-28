.. _use-case-fast-rtps-over-wifi:

Fast DDS over WIFI
===================

The `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ defines the SIMPLE discovery as the default
mechanism for discovering participants in the network.
One of the main features of this mechanism is the use of multicast communication in the Participant Discovery
Phase (PDP).
This could be a problem in case the communication is not wired, i.e. WiFi communication, since multicast is
not as reliable over WiFi as it is over ethernet.
Fast DDS' solution to this challenge is to define the participants with which a unicast communication is to be set, i.e
an initial list of remote peers.

.. _use-case-initial-peers:

Initial Peers
-------------

According to the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ (Section 9.6.1.1), each participant must
listen for incoming PDP discovery metatraffic in two different ports, one linked with a multicast address, and another
one linked to a unicast address.
Fast DDS allows for the configuration of an initial peers list which contains one or more such address-port pairs
corresponding to remote participants PDP discovery listening resources, so that the local participant will not only
send its PDP traffic to the default multicast address-port specified by its domain, but also to all the address-port
pairs specified in the :ref:`Simple Initial Peers` list.

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
| .. literalinclude:: /../code/CodeTester.cpp             |
|    :language: c++                                       |
|    :start-after: //CONF_INITIAL_PEERS_BASIC             |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
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
| .. literalinclude:: /../code/CodeTester.cpp                |
|    :language: c++                                          |
|    :start-after: //CONF_INITIAL_PEERS_METAUNICAST          |
|    :end-before: //!--                                      |
+------------------------------------------------------------+
| **XML**                                                    |
+------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                 |
|    :language: xml                                          |
|    :start-after: <!-->CONF_INITIAL_PEERS_METAUNICAST<-->   |
|    :end-before: <!--><-->                                  |
+------------------------------------------------------------+

