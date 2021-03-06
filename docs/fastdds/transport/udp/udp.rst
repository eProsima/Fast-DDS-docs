.. _transport_udp_udp:

UDP Transport
=============

UDP is a connectionless transport, where the receiving :ref:`dds_layer_domainParticipant` must open a UDP port
listening for incoming messages, and the sending DomainParticipant sends messages to this port.


.. warning::

   This documentation assumes the reader has basic knowledge of UDP/IP concepts, since terms like
   Time To Live (TTL), socket buffers, and port numbering are not explained in detail.
   However, it is possible to configure a basic UDP transport on *Fast DDS* without this knowledge.

.. _transport_udp_transportDescriptor:

UDPTransportDescriptor
----------------------

*eProsima Fast DDS* implements UDP transport for both UDPv4 and UDPv6.
Each of these transports is independent from the other, and has its own :class:`TransportDescriptor`.
However, all their :class:`TransportDescriptor` data members are common.

The following table describes the common data members for both UDPv4 and UDPv6.

.. |InterfaceWhitelist| replace:: :ref:`whitelist-interfaces`

+--------------------------+--------------------+-----------+---------------------------------------------------------+
| Member                   | Data type          | Default   | Description                                             |
+==========================+====================+===========+=========================================================+
| ``sendBufferSize``       | ``uint32_t``       | ``0``     | Size of the sending buffer of the socket (octets).      |
+--------------------------+--------------------+-----------+---------------------------------------------------------+
| ``receiveBufferSize``    | ``uint32_t``       | ``0``     | Size of the receiving buffer of the socket (octets).    |
+--------------------------+--------------------+-----------+---------------------------------------------------------+
| ``interfaceWhiteList``   | ``vector<string>`` | empty     | List of allowed interfaces.                             |
|                          |                    |           | See |InterfaceWhitelist|                                |
+--------------------------+--------------------+-----------+---------------------------------------------------------+
| ``TTL``                  | ``uint8_t``        | ``1``     | Time to live, in number of hops.                        |
+--------------------------+--------------------+-----------+---------------------------------------------------------+
| ``m_output_udp_socket``  | ``uint16_t``       | ``0``     | Port number for the outgoing messages.                  |
+--------------------------+--------------------+-----------+---------------------------------------------------------+
| ``non_blocking_send``    | ``bool``           | ``false`` | Do not block on send operations (*).                    |
+--------------------------+--------------------+-----------+---------------------------------------------------------+

.. note::

   When ``non_blocking_send`` is set to true, send operations will return immediately if the buffer is full, but
   no error will be returned to the upper layer.
   This means that the application will behave as if the datagram is sent and lost.
   This value is specially useful on high-frequency best-effort writers.

   When set to ``false``, send operations will block until the network buffer has space for the
   datagram.
   This may hinder performance on high-frequency writers.


.. _transport_udp_v4transportDescriptor:

UDPv4TransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^

:class:`UDPv4TransportDescriptor` has no additional data members from the common ones described in
:ref:`transport_udp_transportDescriptor`.

.. note::

   The *kind* value for a UDPv4TransportDescriptor is given by the value
   ``eprosima::fastrtps::rtps::LOCATOR_KIND_UDPv4``


.. _transport_udp_v6transportDescriptor:

UDPv6TransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^

:class:`UDPv6TransportDescriptor` has no additional data members from the common ones described in
:ref:`transport_udp_transportDescriptor`.

.. note::

   The *kind* value for a UDPv6TransportDescriptor is given by the value
   ``eprosima::fastrtps::rtps::LOCATOR_KIND_UDPv6``


.. _transport_udp_enabling:

Enabling UDP Transport
----------------------

*Fast DDS* enables a UDPv4 transport by default.
Nevertheless, the application can enable other UDP transports if needed.
To enable a new UDP transport in a :ref:`dds_layer_domainParticipant`, first
create an instance of :ref:`transport_udp_v4transportDescriptor` (for UDPv4) or
:ref:`transport_udp_v6transportDescriptor` (for UDPv6), and add it to the user transport list of the
:ref:`dds_layer_domainParticipant`.

The examples below show this procedure in both C++ code and XML file.

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp   |
|    :language: c++                                |
|    :start-after: //CONF-UDP-TRANSPORT-SETTING    |
|    :end-before: //!--                            |
|    :dedent: 8                                    |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-UDP-TRANSPORT-SETTING |
|    :end-before: <!--><-->                        |
|    :lines: 2-3,5-                                |
|    :append: </profiles>                          |
+--------------------------------------------------+

