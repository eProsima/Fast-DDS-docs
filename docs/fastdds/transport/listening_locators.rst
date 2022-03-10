.. include:: ../../03-exports/aliases-api.include

.. _listening_locators:

Listening Locators
==================

Listening :ref:`Locators<transport_transportApi_locator>` are used to receive incoming traffic on the
:ref:`dds_layer_domainParticipant`.
These Locators can be classified according to the communication type
and to the nature of the data.

According to the communication type we have:

 * **Multicast locators**: Listen to multicast communications.
 * **Unicast locators**: Listen to unicast communications.

According to the nature of the data we have:

 * **Metatraffic locators**: Used to receive metatraffic information, usually used by built-in endpoints to perform
   discovery.
 * **User locators**: Used by the endpoints created by the user to receive user :ref:`dds_layer_topic_topic`
   data changes.

Applications can :ref:`provide their own Listening Locators<listening_locators_adding>`,
or use the :ref:`listening_locators_default` provided by *eProsima Fast DDS*.

.. _listening_locators_adding:

Adding Listening Locators
-------------------------

Users can add custom Listening Locators to the
DomainParticipant using the :ref:`dds_layer_domainParticipantQos`.
Depending on the field where the Locator is added,
it will be treated as a *multicast*, *unicast*, *user* or *metatraffic*
Locator.

.. note::

   Both UDP and TCP unicast Locators support to have a null address.
   In that case, *Fast DDS* automatically gets and uses local network addresses.

.. note::

   Both UDP and TCP Locators support to have a zero port.
   In that case, *Fast DDS* automatically calculates and uses well-known ports for that type of traffic.
   See :ref:`listening_locators_defaultPorts` for details about the well-known ports.

.. warning::

   TCP does not support multicast scenarios, so the network architecture must be carefully planned.

.. _listening_locators_metaMulticast:

Metatraffic Multicast Locators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Users can set their own metatraffic multicast locators within the :ref:`wireprotocolconfigqos`:
|BuiltinAttributes::metatrafficMulticastLocatorList-qos-api|.

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //CONF-TRANSPORT_METAMULTICASTLOCATOR
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->CONF-TRANSPORT_METAMULTICASTLOCATOR
      :end-before: <!--><-->
      :lines: 2-3,5-
      :append: </profiles>

.. _listening_locators_metaUnicast:

Metatraffic Unicast Locators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Users can set their own metatraffic unicast locators within the :ref:`wireprotocolconfigqos`:
|BuiltinAttributes::metatrafficUnicastLocatorList-qos-api|.

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //CONF-TRANSPORT_METAUNICASTLOCATOR
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->CONF-TRANSPORT_METAUNICASTLOCATOR
      :end-before: <!--><-->
      :lines: 2-3,5-
      :append: </profiles>

.. _listening_locators_userMulticast:

User-traffic Multicast Locators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Users can set their own user-traffic multicast locators within the :ref:`wireprotocolconfigqos`:
|WireProtocolConfigQos::default_multicast_locator_list-api|.

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //CONF-TRANSPORT_USERMULTICASTLOCATOR
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->CONF-TRANSPORT_USERMULTICASTLOCATOR
      :end-before: <!--><-->
      :lines: 2-3,5-
      :append: </profiles>

.. _listening_locators_userUnicast:

User-traffic Unicast Locators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Users can set their own user-traffic unicast locators within the :ref:`wireprotocolconfigqos`:
|WireProtocolConfigQos::default_unicast_locator_list-api|.

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //CONF-TRANSPORT_USERUNICASTLOCATOR
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->CONF-TRANSPORT_USERUNICASTLOCATOR
      :end-before: <!--><-->
      :lines: 2-3,5-
      :append: </profiles>

.. _listening_locators_default:

Default Listening Locators
--------------------------

.. _DDSI-RTPS V2.2: https://www.omg.org/spec/DDSI-RTPS/2.2/PDF

If the application does not define any Listening Locators,
*eProsima Fast DDS* automatically enables a set of listening UDPv4 locators by default.
This allows out-of-the-box communication in most cases, without the need of
further configuring the :ref:`comm-transports-configuration`.

 * If the application does not define any *metatraffic* Locator
   (neither *unicast* nor *multicast*), *Fast DDS* enables one *multicast*  Locator
   that will be used during :ref:`discovery`, and one *unicast* Locator
   that will be used for peer-to-peer communication with already discovered
   DomainParticipants.

 * If the application does not define any *user-traffic* Locator
   (neither *unicast* nor *multicast*), *Fast DDS* enables one *unicast* Locator
   that will be used for peer-to-peer communication of :ref:`dds_layer_topic_topic` data.

For example, it is possible to prevent *multicast* traffic adding a single *user-traffic unicast* Locator
as described in :ref:`transport_disableMulticast`.

Default Listening Locators always use :ref:`listening_locators_defaultPorts`.

.. _listening_locators_defaultPorts:

Well Known Ports
----------------

The `DDSI-RTPS V2.2`_ standard (Section 9.6.1.1) defines a set of rules to calculate well-known
ports for default Locators, so that DomainParticipants can communicate with these default Locators.
Well-known ports are also selected automatically by *Fast DDS* when a Locator is configured with port number `0`.

Well-known ports are calculated using the following predefined rules:

.. list-table:: Rules to calculate ports on default listening locators
   :header-rows: 1

   * - Traffic type
     - Well-known port expression
   * - Metatraffic multicast
     - ``PB`` + ``DG`` * *domainId* + ``offsetd0``
   * - Metatraffic unicast
     - ``PB`` + ``DG`` * *domainId* + ``offsetd1`` + ``PG`` * *participantId*
   * - User multicast
     - ``PB`` + ``DG`` * *domainId* + ``offsetd2``
   * - User unicast
     - ``PB`` + ``DG`` * *domainId* + ``offsetd3`` + ``PG`` * *participantId*


The values used in these rules are explained on the following table.
The default values can be modified using the |WireProtocolConfigQos::port-api| member of the
:ref:`wireprotocolconfigqos` on the :ref:`dds_layer_domainParticipantQos`.

.. list-table:: Values used in the rules to calculate well-known ports
   :header-rows: 1

   * - Symbol
     - Meaning
     - Default value
     - QoS field
   * - ``DG``
     - DomainID gain
     - ``250``
     - |PortParameters::domainIDGain-qos-api|
   * - ``PG``
     - ParticipantId gain
     - ``2``
     - |PortParameters::participantIDGain-qos-api|
   * - ``PB``
     - Port Base number
     - ``7400``
     - |PortParameters::portBase-qos-api|
   * - ``offsetd0``
     - Additional offset
     - ``0``
     - |PortParameters::offsetd0-qos-api|
   * - ``offsetd1``
     - Additional offset
     - ``10``
     - |PortParameters::offsetd1-qos-api|
   * - ``offsetd2``
     - Additional offset
     - ``1``
     - |PortParameters::offsetd2-qos-api|
   * - ``offsetd3``
     - Additional offset
     - ``11``
     - |PortParameters::offsetd3-qos-api|
