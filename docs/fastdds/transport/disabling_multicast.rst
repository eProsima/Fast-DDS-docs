.. _transport_disableMulticast:

Disabling all Multicast Traffic
===============================

If all the peers are known beforehand and have been configured on the
:ref:`intial peers list<Simple Initial Peers>`, all multicast traffic can be completely disabled.

By defining a custom :ref:`listening_locators_metaUnicast`, the local :ref:`dds_layer_domainParticipant`
creates a unicast meta traffic receiving resource for each address-port pair specified in the list,
avoiding the creation of the default metatraffic multicast and unicast locators.
This prevents the :ref:`dds_layer_domainParticipant` from listening to any discovery data from
multicast sources.

Consideration should be given to the assignment of the ports in the ``metatrafficUnicastLocatorList``,
avoiding the assignment of ports that are not available or do not match the address-port
listed in the publisher participant :ref:`intial peers list<Simple Initial Peers>`.

The following is an example of how to disable all multicast traffic configuring one
*metatraffic unicast* locator.

+-----------------------------------------------+
| **C++**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp   |
|    :language: c++                             |
|    :start-after: //CONF-DISABLE-MULTICAST     |
|    :end-before: //!--                         |
+-----------------------------------------------+
| **XML**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml    |
|    :language: xml                             |
|    :start-after: <!-->CONF-DISABLE-MULTICAST  |
|    :end-before: <!--><-->                     |
+-----------------------------------------------+



