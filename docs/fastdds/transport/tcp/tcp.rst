.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _transport_tcp_tcp:

TCP Transport
=============

TCP is a connection oriented transport, so the :ref:`dds_layer_domainParticipant` must establish a TCP connection
to the remote peer before sending data messages.
Therefore, one of the communicating DomainParticipants (the one acting as *server*) must open a TCP port listening for
incoming connections, and the other one (the one acting as *client*) must connect to this port.

.. note::

  The *server* and *client* concepts are independent from the DDS concepts of
  :ref:`dds_layer_publisher`, :ref:`dds_layer_subscriber`,
  :ref:`dds_layer_publisher_dataWriter`, and :ref:`dds_layer_subscriber_dataReader`.
  Also, these concepts are independent from the *eProsima Discovery Server* *servers* and *clients*
  (:ref:`discovery_server`).
  Any of them can act as a *TCP Server* or *TCP Client* when establishing the connection,
  and the DDS communication will work over this connection.

.. warning::

   This documentation assumes the reader has basic knowledge of TCP/IP concepts, since terms like
   Time To Live (TTL), Cyclic Redundancy Check (CRC), Transport Layer Security (TLS),
   socket buffers, and port numbering are not explained in detail.
   However, it is possible to configure a basic TCP transport on *Fast DDS* without this knowledge.


.. _transport_tcp_transportDescriptor:

TCPTransportDescriptor
----------------------

*eProsima Fast DDS* implements TCP transport for both TCPv4 and TCPv6.
Each of these transports is independent from the other, and has its own |TransportDescriptorInterface-api|.
However, they share many of their features, and most of the |TransportDescriptorInterface-api| data members are common.

The following table describes the common data members for both TCPv4 and TCPv6.

.. |InterfaceWhitelist| replace:: :ref:`whitelist-interfaces`
.. |TLSconfig| replace:: :ref:`transport_tcp_tls`

.. list-table::
  :header-rows: 1
  :align: left

  * - Member
    - Data type
    - Default
    - Description
  * - |SocketTransportDescriptor::sendBufferSize-api|
    - ``uint32_t``
    - 0
    - Size of the sending buffer of the socket (octets).
  * - |SocketTransportDescriptor::receiveBufferSize-api|
    - ``uint32_t``
    - 0
    - Size of the receiving buffer of the socket (octets).
  * - |SocketTransportDescriptor::interfaceWhiteList-api|
    - ``vector<string>``
    - Empty vector
    - List of allowed interfaces
      See |InterfaceWhitelist|.
  * - |SocketTransportDescriptor::TTL-api|
    - ``uint8_t``
    - 1
    - Time to live, in number of hops.
  * - |TCPTransportDescriptor::listening_ports-api|
    - ``vector<uint16_t>``
    - Empty vector
    - List of ports to listen as *server*. If a port is set to 0, an available port will be automatically assigned.
  * - |TCPTransportDescriptor::keep_alive_frequency_ms-api|
    - ``uint32_t``
    - 5000
    - Frequency of RTCP keep alive requests (in ms).
  * - |TCPTransportDescriptor::keep_alive_timeout_ms-api|
    - ``uint32_t``
    - 15000
    - Time since sending the last keep alive request to consider a connection as broken (in ms).
  * - |TCPTransportDescriptor::max_logical_port-api|
    - ``uint16_t``
    - 100
    - Maximum number of logical ports to try during RTCP negotiation.
  * - |TCPTransportDescriptor::logical_port_range-api|
    - ``uint16_t``
    - 20
    - Maximum number of logical ports per request to try during RTCP negotiation.
  * - |TCPTransportDescriptor::logical_port_increment-api|
    - ``uint16_t``
    - 2
    - Increment between logical ports to try during RTCP negotiation.
  * - |TCPTransportDescriptor::enable_tcp_nodelay-api|
    - ``bool``
    - ``false``
    - Enables the TCP_NODELAY socket option.
  * - |TCPTransportDescriptor::calculate_crc-api|
    - ``bool``
    - ``true``
    - True to calculate and send CRC on message headers.
  * - |TCPTransportDescriptor::check_crc-api|
    - ``bool``
    - ``true``
    - True to check the CRC of incoming message headers.
  * - |TCPTransportDescriptor::apply_security-api|
    - ``bool``
    - ``false``
    - True to use TLS. See |TLSconfig|.
  * - |TCPTransportDescriptor::tls_config-api|
    - |TCPTransportDescriptor::TLSConfig-api|
    -
    - Configuration for TLS. See |TLSconfig|.
  * - |PortBasedTransportDescriptor::default_reception_threads-api|
    - |ThreadSettings|
    -
    - Default |ThreadSettings| for the reception threads.
  * - |PortBasedTransportDescriptor::reception_threads-api|
    - ``std::map<uint32_t, ThreadSettings>``
    -
    - |ThreadSettings| for the reception threads on specific ports.
  * - |TCPTransportDescriptor::keep_alive_thread-api|
    - |ThreadSettings|
    -
    - |ThreadSettings| for the thread keeping alive TCP connections.
  * - |TCPTransportDescriptor::accept_thread-api|
    - |ThreadSettings|
    -
    - |ThreadSettings| for the threads processing incoming TCP connection requests.

.. note::

  If |TCPTransportDescriptor::listening_ports-api| is left empty, the participant will not be able to receive incoming
  connections but will be able to connect to other participants that have configured their listening ports.

.. _transport_tcp_v4transportDescriptor:

TCPv4TransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^

The following table describes the data members that are exclusive for |TCPv4TransportDescriptor-api|.

.. |WANconfig| replace:: :ref:`transport_tcp_wan`

.. list-table::
  :header-rows: 1
  :align: left

  * - Member
    - Data type
    - Default
    - Description
  * - |TCPv4TransportDescriptor::wan_addr-api|
    - ``octet[4]``
    - [0, 0, 0, 0]
    - Configuration for WAN. See |WANconfig|.

.. note::

   The |TransportInterface::kind-api| value for a |TCPv4TransportDescriptor-api| is given by the value
   |LOCATOR_KIND_TCPv4-api|.


.. _transport_tcp_v6transportDescriptor:

TCPv6TransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^

|TCPv6TransportDescriptor-api| has no additional data members from the common ones described in
:ref:`transport_tcp_transportDescriptor`.

.. note::

   The |TransportInterface::kind-api| value for a |TCPv6TransportDescriptor-api| is given by the value
   |LOCATOR_KIND_TCPv6-api|.

.. _transport_tcp_enabling:

Enabling TCP Transport
----------------------

To enable TCP transport in a DomainParticipant, you need to
create an instance of :ref:`transport_tcp_v4transportDescriptor` (for TCPv4) or
:ref:`transport_tcp_v6transportDescriptor` (for TCPv6), and add it to the user transport list of the
DomainParticipant.

Depending on the TCP transport descriptor settings and network locators defined, the DomainParticipant can act as a
TCP Server or TCP Client.

* **TCP Server**: If you provide |TCPTransportDescriptor::listening_ports-api| on the descriptor, the DomainParticipant
  will act as *TCP server*, listening for incoming remote connections on the given ports.
  The examples below show this procedure in both C++ code and XML file.

  .. tabs::

    .. tab:: C++

      .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-TCP-TRANSPORT-SETTING-SERVER
        :end-before: //!--
        :dedent: 8

    .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-SERVER
        :end-before: <!--><-->
        :lines: 2-4,6-73,75-76

* **TCP Client**: If you provide |BuiltinAttributes::initialPeersList-api| to the DomainParticipant, it will act as
  *TCP client*, trying to connect to the remote *servers* at the given addresses and ports.
  The examples below show this procedure in both C++ code and XML file.
  See :ref:`Simple Initial Peers` for more information about their configuration.

  .. tabs::

    .. tab:: C++

      .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-TCP-TRANSPORT-SETTING-CLIENT
        :end-before: //!--
        :dedent: 8

    .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-CLIENT
        :end-before: <!--><-->
        :lines: 2-4,6-58,60-61

:ref:`transport_tcp_example` shows how to use and configure a TCP transport.

.. _transport_tcp_wan:

WAN or Internet Communication over TCPv4
----------------------------------------

*Fast DDS* is able to connect through the Internet or other WAN networks when configured properly.
To achieve this kind of scenarios, the involved network devices such as routers and firewalls
must add the rules to allow the communication.

For example, imagine we have the scenario represented on the following figure:

.. image:: /01-figures/TCP_WAN.png
    :align: center

* A DomainParticipant acts as a *TCP server* listening on port ``5100``
  and is connected to the WAN through a router with public IP ``80.80.99.45``.

* Another DomainParticipant acts as a *TCP client* and has configured
  the server's IP address and port in its :ref:`Simple Initial Peers` list.

On the server side, the router must be configured to forward to the *TCP server*
all traffic incoming to port ``5100``. Typically, a NAT routing of port ``5100`` to our
machine is enough. Any existing firewall should be configured as well.

In addition, to allow incoming connections through a WAN,
the :ref:`transport_tcp_v4transportDescriptor` must indicate its **public** IP address
in the |TCPv4TransportDescriptor::wan_addr-api| data member.
The following examples show how to configure the DomainParticipant both in C++ and XML.

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //CONF-TCP-TRANSPORT-SETTING-SERVER
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-SERVER
      :end-before: <!--><-->
      :lines: 2-4,6-73,75-76

On the client side, the DomainParticipant must be configured
with the **public** IP address and |TCPTransportDescriptor::listening_ports-api| of the *TCP server* as
:ref:`Simple Initial Peers`.

.. tabs::

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //CONF-TCP-TRANSPORT-SETTING-CLIENT
      :end-before: //!--
      :dedent: 8

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-CLIENT
      :end-before: <!--><-->
      :lines: 2-4,6-58,60-61

.. _transport_tcp_example:

HelloWorldExampleTCP
--------------------

A TCP version of helloworld example can be found in the
`HelloWorldExampleTCP folder <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/dds/HelloWorldExampleTCP>`_.
It shows a publisher and a subscriber that communicate through TCP.
The publisher is configured as *TCP server* while the Subscriber is acting as *TCP client*.



