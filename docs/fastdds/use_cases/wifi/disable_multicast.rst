.. _use-case-disabling-multicast-discovery:

Disabling multicast discovery
=============================

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
|    :lines: 2-3,5-                                          |
|    :append: </profiles>                                    |
+------------------------------------------------------------+

