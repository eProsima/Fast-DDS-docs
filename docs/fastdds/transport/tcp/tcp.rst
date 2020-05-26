.. _transport_tcp_tcp:

TCP Transport
=============

TCP is a connection oriented transport, so the :ref:`dds_layer_domainParticipant` must establish a TCP connection
to the remote peer before sending data messages.
Therefore, one of the communicating :ref:`DomainParticipants<dds_layer_domainParticipant>` (the one acting
as *server*) must open a TCP port listening for incoming connections, and the other one (the one acting as *client*)
must connect to this port.

.. note::

  The *server* and *client* concepts are independent from the DDS concepts of
  :ref:`dds_layer_publisher`, :ref:`dds_layer_subscriber`,
  :ref:`dds_layer_publisher_dataWriter`, and :ref:`dds_layer_subscriber_dataReader`.
  Any of them can act as a *TCP Server* or *TCP Client* when establishing the connection,
  and the DDS communication will work over this connection.


.. _transport_tcp_enabling:

Enabling TCP Transport
----------------------

To enable TCP transport in a :ref:`dds_layer_domainParticipant`, you need to
create an instance of :ref:`transport_tcp_v4transportDescriptor` (for TCPv4) or
:ref:`transport_tcp_v6transportDescriptor` (for TCPv6), and add it to the user transport list of the
:ref:`dds_layer_domainParticipant`.

If you provide ``listening_ports`` on the descriptor, the :ref:`dds_layer_domainParticipant` will act
as *TCP server*, listening for incoming remote connections on the given ports.
The examples below show this procedure in both C++ code and XML file.

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

If you provide ``initialPeersList`` to the :ref:`dds_layer_domainParticipant`, it will act
as *TCP client*, trying to connect to the remote *servers* at the given addresses and ports.
The examples below show this procedure in both C++ code and XML file.
See :ref:`initial-peers` for more information about their configuration.

+---------------------------------------------------+
| **C++**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp       |
|    :language: c++                                 |
|    :start-after: //CONF-TCP2-TRANSPORT-SETTING    |
|    :end-before: //!--                             |
+---------------------------------------------------+
| **XML**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml        |
|    :language: xml                                 |
|    :start-after: <!-->CONF-TCP2-TRANSPORT-SETTING |
|    :end-before: <!--><-->                         |
+---------------------------------------------------+

There is an :ref:`example<transport_tcp_example>` that shows the use and configuration of TCP transport.


.. _transport_tcp_transportDescriptor:

TCPTransportDescriptor
----------------------

eProsima Fast DDS implements TCP transport for both TCPv4 and TCPv6.
Each of these transports is independent from the other, and has its own :class:`TransportDescriptor`.
However, they share many of their features, and most of the :class:`TransportDescriptor` data members are common.

The following table describes the common data members for both TCPv4 and TCPv6.

.. |InterfaceWhitelist| replace:: :ref:`whitelist-interfaces`
.. |TLSconfig| replace:: :ref:`transport_tcp_tls`

+------------------------------+----------------------+---------+------------------------------------------------------+
| Member                       | Data type            | Default | Description                                          |
+==============================+======================+=========+======================================================+
| ``sendBufferSize``           | ``uint32_t``         | ``0``   | Size of the sending buffer of the socket.            |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``receiveBufferSize``        | ``uint32_t``         | ``0``   | Size of the receiving buffer of the socket.          |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``interfaceWhiteList``       | ``vector<string>``   | empty   | List of allowed interfaces.                          |
|                              |                      |         | See |InterfaceWhitelist|                             |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``TTL``                      | ``uint8_t``          | ``1``   | Time to live, in number of hops.                     |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``listening_ports``          | ``vector<uint16_t>`` | empty   | List of ports to listen as *server*.                 |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``keep_alive_frequency_ms``  | ``uint32_t``         | 5000    | Sending frequency of RTCP keepalive requests (in ms).|
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``keep_alive_timeout_ms``    | ``uint32_t``         | 15000   | Time since sending the last keepalive request to     |
|                              |                      |         | consider a connection as broken (in ms).             |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``max_logical_port``         | ``uint16_t``         | 100     | Maximum number of logical ports to try               |
|                              |                      |         | during RTCP negotiation.                             |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``logical_port_range``       | ``uint16_t``         | 20      | Maximum number of logical ports per request to try   |
|                              |                      |         | during RTCP negotiation.                             |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``logical_port_increment``   | ``uint16_t``         | 2       | Increment between logical ports to try               |
|                              |                      |         | during RTCP negotiation.                             |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``tcp_negotiation_timeout``  | ``uint32_t``         | 5000    | Timeout for the health check of ports.               |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``enable_tcp_nodelay``       | ``bool``             | false   | Enables the TCP_NODELAY socket option.               |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``calculate_crc``            | ``bool``             | true    | True to calculate and send CRC on message headers.   |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``check_crc``                | ``bool``             | true    | True to check the CRC of incomming message headers.  |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``apply_security``           | ``bool``             | false   | True to use TLS. See |TLSconfig|.                    |
+------------------------------+----------------------+---------+------------------------------------------------------+
| ``tls_config``               | ``TLSConfig``        |         | Configuration for TLS. See |TLSconfig|.              |
+------------------------------+----------------------+---------+------------------------------------------------------+

.. note::

  If ``listening_ports`` is left empty, the participant will not be able to receive incoming connections but will be able
  to connect to other participants that have configured their listening ports.

.. _transport_tcp_v4transportDescriptor:

TCPv4TransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^

The following table describes the data members that are exclusive for :class:`TCPv4TransportDescriptor`.

.. |WANconfig| replace:: :ref:`transport_tcp_wan`

+------------------------------+----------------------+---------+------------------------------------------------------+
| Member                       | Data type            | Default | Description                                          |
+==============================+======================+=========+======================================================+
| ``wan_addr``                 | ``octet[4]``         | empty   | Configuration for TLS. See |WANconfig|.              |
+------------------------------+----------------------+---------+------------------------------------------------------+


.. _transport_tcp_v6transportDescriptor:

TCPv6TransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^

:class:`TCPv6TransportDescriptor` has no additional data members from the common ones described in
:ref:`transport_tcp_transportDescriptor`.



.. _transport_tcp_example:

HelloWorldExampleTCP
--------------------

A TCP version of helloworld example can be found in the ``examples/C++/DDS/HelloWorldExampleTCP`` folder.
It shows a publisher and a subscriber that communicate through TCP.
The publisher is configured as *TCP server* while the Subscriber is acting as *TCP client*.














.. _transport_tcp_wan:

WAN or Internet Communication over TCP/IPv4
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast-RTPS is able to connect through the Internet or other WAN networks when configured properly.
To achieve this kind of scenarios, the involved network devices such as routers and firewalls
should add the rules to allow the communication.

For example, to allow incoming connections through our NAT, Fast-RTPS must be configured as a **TCP Server** listening
to incoming TCP connections.
To allow incoming connections through a WAN, the TCP descriptor associated must indicate
its public IP through its field ``wan_addr``.

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

In this case, configuring the router (which public IP is ``80.80.99.45``) is mandatory to allow the incoming traffic to
reach the **TCP Server**.
Typically a NAT routing with the ``listening_port`` ``5100`` to our machine is enough.
Any existing firewall should be configured as well.

In the client side, it is needed to specify the public IP of the **TCP Server** with its ``listening_port`` as
``initial_peer``.

+---------------------------------------------------+
| **C++**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp       |
|    :language: c++                                 |
|    :start-after: //CONF-TCP2-TRANSPORT-SETTING    |
|    :end-before: //!--                             |
+---------------------------------------------------+
| **XML**                                           |
+---------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml        |
|    :language: xml                                 |
|    :start-after: <!-->CONF-TCP2-TRANSPORT-SETTING |
|    :end-before: <!--><-->                         |
+---------------------------------------------------+

The combination of the above configurations in both **TCP Server** and **TCP Client** allows a scenario similar to
the represented by the following image.

.. image:: /01-figures/TCP_WAN.png
    :align: center

**IPLocator**

IPLocator is an auxiliary static class that offers methods to ease the management of IP based locators, as UDP or TCP.
In TCP, the port field of the locator is divided into physical and logical port.
The physical port is the port used by the network device, the real port that the operating system understands.
The logical port can be seen as RTPS port, or UDP's equivalent port (physical ports of UDP, are logical ports in TCP).
Logical ports normally are not necessary to manage explicitly, but you can do it through IPLocator class.
Physical ports instead, must be set to explicitly use certain ports, to allow the communication through a NAT, for
example.

.. literalinclude:: /../code/CodeTester.cpp
    :language: c++
    :start-after: //CONF-IPLOCATOR-USAGE
    :end-before: //!--

**NOTE**

TCP doesn't support multicast scenarios, so you must plan carefully your network architecture.


.. _transport_tcp_tls:

TLS over TCP
^^^^^^^^^^^^

Fast-RTPS allows configuring a TCP Transport to use TLS (Transport Layer Security)
by setting up **TCP Server** and **TCP Client** properly.

 **TCP Server**

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-TCP-TLS-SERVER           |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-TCP-TLS-SERVER        |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

 **TCP Client**

+------------------------------------------------------+
| **C++**                                              |
+------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp          |
|    :language: c++                                    |
|    :start-after: //CONF-TCP-TLS-CLIENT               |
|    :end-before: //!--                                |
+------------------------------------------------------+
| **XML**                                              |
+------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml           |
|    :language: xml                                    |
|    :start-after: <!-->CONF-TCP-TLS-CLIENT            |
|    :end-before: <!--><-->                            |
+------------------------------------------------------+

More TLS related options can be found in the section :ref:`transportdescriptors`.
