.. _listening_locators:

Listening locators
------------------

*eProsima Fast RTPS* divides listening locators into four categories:

* Metatraffic Multicast Locators: these locators are used to receive metatraffic information using multicast.
  They usually are used by built-in endpoints, like the discovery of built-in endpoints. You can set your own locators
  using attribute ``rtps.builtin.metatrafficMulticastLocatorList``.

  .. literalinclude:: /../code/CodeTester.cpp
      :language: c++
      :start-after: //CONF-METAMULTICASTLOCATOR
      :end-before: //!--

* Metatraffic Unicast Locators: these locators are used to receive metatraffic information using unicast.
  They usually are used by built-in endpoints, like the discovery of built-in endpoints.
  You can set your own locators using attribute ``rtps.builtin.metatrafficUnicastLocatorList``.

  .. literalinclude:: /../code/CodeTester.cpp
      :language: c++
      :start-after: //CONF-METAUNICASTLOCATOR
      :end-before: //!--

* User Multicast Locators: these locators are used to receive user information using multicast. They are used by user
  endpoints. You can set your own locators using attribute ``rtps.defaultMulticastLocatorList``.

  .. literalinclude:: /../code/CodeTester.cpp
      :language: c++
      :start-after: //CONF-USERMULTICASTLOCATOR
      :end-before: //!--

* User Unicast Locators: these locators are used to receive user information using unicast. They are used by user
  endpoints. You can set your own locators using attributes ``rtps.defaultUnicastLocatorList``.

  .. literalinclude:: /../code/CodeTester.cpp
      :language: c++
      :start-after: //CONF-USERUNICASTLOCATOR
      :end-before: //!--

By default *eProsima Fast RTPS* calculates the listening locators for the built-in UDPv4 network transport using
well-known ports. These well-known ports are calculated using the following predefined rules:

.. list-table:: Ports used
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

These predefined rules use some values explained here:

* DG: DomainId Gain. You can set this value using attribute ``rtps.port.domainIDGain``.
* PG: ParticipantId Gain. You can set this value using attribute ``rtps.port.participantIDGain``.
  The default value is ``2``.
* PB: Port Base number. You can set this value using attribute ``rtps.port.portBase``.
  The default value is ``7400``.
* offsetd0, offsetd1, offsetd2, offsetd3: Additional offsets.
  You can set these values using attributes
  ``rtps.port.offsetdN``. Default values are: ``offsetd0 = 0``, ``offsetd1 = 10``, ``offsetd2 = 1``, ``offsetd3 = 11``.

Both UDP and TCP unicast locators support to have a null address.
In that case, *eProsima Fast RTPS* understands to get local network addresses and use them.

Both UDP and TCP locators support to have a zero port.
In that case, *eProsima Fast RTPS* understands to calculate well-known port for that type of traffic.
