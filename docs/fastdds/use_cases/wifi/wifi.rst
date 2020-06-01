.. _use-case-fast-rtps-over-wifi:

Fast DDS over WIFI
==================

.. _RTPS v2.2 standard: https://www.omg.org/spec/DDSI-RTPS/2.2/

The `RTPS v2.2 standard`_ defines the SIMPLE :ref:`discovery` as the default
mechanism for discovering participants in the network.
One of the main features of this mechanism is the use of multicast communication in the Participant Discovery
Phase (PDP).
This can be a problem in cases where WiFi communication is used, since multicast is not as reliable over WiFi
as it is over ethernet.

The recommended solution to this challenge is to configure an initial list of remote peers on the
:ref:`dds_layer_domainParticipant`, so that it can set unicast communication with them.
This way, the use of multicast is not needed to discover this initial peers.
Furthermore, if all the peers are known and configured beforehand, all multicast communication can be
removed.

.. _use-case-initial-peers:

Configuring Initial Peers
-------------------------

A complete description of the initial peers list and its configuration can be found on
:ref:`Simple Initial Peers`.
For convenience, however, this example shows how to configure an initial peers list with one peer
on host ``192.168.10.13`` with participant ID ``1`` in domain ``0``.
Beware that the port number used here is not arbitrary, as discovery ports are defined by
the `RTPS v2.2 standard`_.
Refer to :ref:`listening_locators_defaultPorts` to learn about these standard port numbers.


+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp          |
|    :language: c++                                       |
|    :start-after: //CONF_INITIAL_PEERS_BASIC             |
|    :end-before: //!--                                   |
|    :dedent: 8                                           |
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

If all the peers are known and configured on the initial peer list beforehand,
it is possible to disable the multicast meta traffic completely, as all
:ref:`DomainParticipants<dds_layer_domainParticipant>` can communicate among them through unicast.

The complete description of the procedure to disable multicast discovery can be found at
:ref:`transport_disableMulticast`.
For convenience, however, this example shows how to disable all multicast traffic configuring one
*metatraffic unicast* locator.
Consideration should be given to the assignment of the ports in the ``metatrafficUnicastLocatorList``,
avoiding the assignment of ports that are not available or do not match the address-port
listed in the :ref:`intial peers list<Simple Initial Peers>` of the peer participant.

+------------------------------------------------------------+
| **C++**                                                    |
+------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp             |
|    :language: c++                                          |
|    :start-after: //CONF_INITIAL_PEERS_METAUNICAST          |
|    :end-before: //!--                                      |
|    :dedent: 8                                              |
+------------------------------------------------------------+
| **XML**                                                    |
+------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                 |
|    :language: xml                                          |
|    :start-after: <!-->CONF_INITIAL_PEERS_METAUNICAST<-->   |
|    :end-before: <!--><-->                                  |
+------------------------------------------------------------+



