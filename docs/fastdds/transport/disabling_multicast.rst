.. include:: ../../03-exports/aliases-api.include

.. _transport_disableMulticast:

Disabling all Multicast Traffic
===============================

If all the peers are known beforehand and have been configured on the
:ref:`Initial Peers List<Simple Initial Peers>`, all multicast traffic can be completely disabled.

By defining a custom :ref:`listening_locators_metaUnicast`, the local :ref:`dds_layer_domainParticipant`
creates a unicast meta traffic receiving resource for each address-port pair specified in the list,
avoiding the creation of the default metatraffic multicast and unicast locators.
This prevents the DomainParticipant from listening to any discovery data from
multicast sources.

Consideration should be given to the assignment of the ports in the
|BuiltinAttributes::metatrafficUnicastLocatorList-api|, avoiding the assignment of ports that are not available or do
not match the address-port listed in the publisher participant Initial Peers List.

The following is an example of how to disable all multicast traffic configuring one
*metatraffic unicast* locator.

.. tab-set::

  .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../code/DDSCodeTester.cpp
          :language: c++
          :start-after: //CONF-DISABLE-MULTICAST
          :end-before: //!--
          :dedent: 8

  .. tab-item:: XML
      :sync: xml

      .. literalinclude:: /../code/XMLTester.xml
          :language: xml
          :start-after: <!-->CONF-DISABLE-MULTICAST
          :end-before: <!--><-->
          :lines: 2-3,5-
          :append: </profiles>
