.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _whitelist-interfaces:

Interface Whitelist
-------------------

Using *Fast DDS*, it is possible to limit the network interfaces used by :ref:`transport_tcp_tcp` and
:ref:`transport_udp_udp`.
This is achieved by adding the interfaces to the |SocketTransportDescriptor::interfaceWhiteList-api|
field in the :ref:`transport_tcp_transportDescriptor` or :ref:`transport_udp_transportDescriptor`.
Thus, the communication interfaces used by the |DomainParticipants| whose |TransportDescriptorInterface-api| defines an
|SocketTransportDescriptor::interfaceWhiteList-api| is limited to the interfaces' addresses defined in that list,
therefore avoiding the use of the rest of the network interfaces available in the system.
It is possible to add the interfaces to |SocketTransportDescriptor::interfaceWhiteList-api| specifying the IP addresses
or the interface names.
For example:

* Interface whitelist filled with IP address:

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //TRANSPORT-DESCRIPTORS
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->TRANSPORT-DESCRIPTORS
      :end-before: <!--><-->
      :lines: 2-3,5-
      :append: </profiles>

* Interface whitelist filled with interface names:

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //WHITELIST-NAME
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->WHITELIST-NAME
      :end-before: <!--><-->
      :lines: 2-3,5-
      :append: </profiles>

.. important::

  If none of the values in the transport descriptor's whitelist match the interfaces on your machine,
  all the interfaces in the whitelist are filtered out and no communication will be established through that transport.

.. warning::

  The interface whitelist feature applies to network interfaces.
  Therefore, it is only available on :ref:`transport_tcp_tcp` and :ref:`transport_udp_udp`.

