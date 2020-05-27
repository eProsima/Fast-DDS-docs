.. _listening_locators:

Listening Locators
==================

Listening :ref:`Locators<transport_transportApi_locator>` are used to receive incoming traffic on the
:ref:`dds_layer_domainParticipant`.
These :ref:`Locators<transport_transportApi_locator>` can be classified according to the communication type
and to the nature of the data.

According to the communication type we have:

 * **Multicast locators**: Listen to multicast communications.
 * **Unicast locators**: Listen to unicast communications.

According to the nature of the data we have:

 * **Metatraffic locators**: Used to receive metatraffic information, usually used by built-in endpoints to perform
   discovery.
 * **User locators**: Used by the endpoints created by the user to receive user :ref:`dds_layer_topic_topic`
   data changes.


.. _listening_locators_adding:

Adding Listening Locators
-------------------------

Users can add Listening :ref:`Locators<transport_transportApi_locator>` to the :ref:`dds_layer_domainParticipant`
using the :ref:`dds_layer_domainParticipantQos`.
Depending on the field of the :ref:`dds_layer_domainParticipant` where the :ref:`transport_transportApi_locator`
is added, it will be treated as a *multicast*, *unicast*, *user* or *metatraffic*
:ref:`transport_transportApi_locator`.

.. note::

   Both UDP and TCP unicast :ref:`Locators<transport_transportApi_locator>` support to have a null address.
   In that case, Fast DDS automatically gets and uses local network addresses.

.. note::

   Both UDP and TCP :ref:`Locators<transport_transportApi_locator>` support to have a zero port.
   In that case, Fast DDS automatically calculates and uses well-known ports for that type of traffic.
   See :ref:`listening_locators_default` for details about the well-known ports.

.. note::

   TCP does not support multicast scenarios, so the network architecture must be carefully planned.

.. _listening_locators_metaMulticast:

Metatraffic Multicast Locators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Users can set their own metatraffic multicast locators
using the field ``wire_protocol().builtin.metatrafficMulticastLocatorList``.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_TRANSPORT_METAMULTICASTLOCATOR
    :end-before: //!--

.. _listening_locators_metaUnicast:

Metatraffic Unicast Locators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Users can set their own metatraffic unicast locators
using the field ``wire_protocol().builtin.metatrafficUnicastLocatorList``.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_TRANSPORT_METAUNICASTLOCATOR
    :end-before: //!--

.. _listening_locators_userMulticast:

User Multicast Locators
^^^^^^^^^^^^^^^^^^^^^^^
Users can set their own user multicast locators
using the field ``wire_protocol().default_multicast_locator_list``.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_TRANSPORT_USERMULTICASTLOCATOR
    :end-before: //!--

.. _listening_locators_userUnicast:

User Unicast Locators
^^^^^^^^^^^^^^^^^^^^^
Users can set their own user unicast locators
using the field ``wire_protocol().default_unicast_locator_list``.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_TRANSPORT_USERUNICASTLOCATOR
    :end-before: //!--


.. _listening_locators_default:

Default Listening Locators
--------------------------

eProsima Fast DDS enables a set of listening UDPv4 locators by default, using well-known ports.
This allows out-of-the-box communication in most cases, without the need of configuring the
:ref:`comm-transports-configuration` layer.

The well-known ports are calculated using the following predefined rules:

.. list-table:: Rules to calculate ports on default listening locators
   :header-rows: 1

   * - Traffic type
     - Well-known port expression
   * - Metatraffic multicast
     - PB + DG * *domainId* + offsetd0
   * - Metatraffic unicast
     - PB + DG * *domainId* + offsetd1 + PG * *participantId*
   * - User multicast
     - PB + DG * *domainId* + offsetd2
   * - User unicast
     - PB + DG * *domainId* + offsetd3 + PG * *participantId*


The values used in these rules are explained on the following table.
The default values can be modified using the corresponding field on the :ref:`dds_layer_domainParticipantQos`.

.. list-table:: Values used in the rules to calculate well-known ports
   :header-rows: 1

   * - Symbol
     - Meaning
     - Default value
     - QoS field
   * - ``DG``
     - DomainID gain
     - ``250``
     - ``wire_protocol().port.domainIDGain``
   * - ``PG``
     - ParticipantId gain
     - ``2``
     - ``wire_protocol().port.participantIDGain``
   * - ``PB``
     - Port Base number
     - ``7400``
     - ``wire_protocol().port.portBase``
   * - ``offsetd0``
     - Additional offset
     - ``0``
     - ``wire_protocol().port.offsetd0``
   * - ``offsetd1``
     - Additional offset
     - ``10``
     - ``wire_protocol().port.offsetd1``
   * - ``offsetd2``
     - Additional offset
     - ``1``
     - ``wire_protocol().port.offsetd2``
   * - ``offsetd3``
     - Additional offset
     - ``11``
     - ``wire_protocol().port.offsetd3``




