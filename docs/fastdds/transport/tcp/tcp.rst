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

+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp          |
|    :language: c++                                       |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING-SERVER    |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
|    :language: xml                                       |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-SERVER |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

If you provide ``initialPeersList`` to the :ref:`dds_layer_domainParticipant`, it will act
as *TCP client*, trying to connect to the remote *servers* at the given addresses and ports.
The examples below show this procedure in both C++ code and XML file.
See :ref:`initial-peers` for more information about their configuration.

+----------------------------------------------------------+
| **C++**                                                  |
+----------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp           |
|    :language: c++                                        |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING-CLIENT     |
|    :end-before: //!--                                    |
+----------------------------------------------------------+
| **XML**                                                  |
+----------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml               |
|    :language: xml                                        |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-CLIENT  |
|    :end-before: <!--><-->                                |
+----------------------------------------------------------+

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


.. _transport_tcp_wan:

WAN or Internet Communication over TCPv4
----------------------------------------

Fast DDS is able to connect through the Internet or other WAN networks when configured properly.
To achieve this kind of scenarios, the involved network devices such as routers and firewalls
must add the rules to allow the communication.

For example, imagine we have the scenario repesented on the following figure:

.. image:: /01-figures/TCP_WAN.png
    :align: center

* A :ref:`dds_layer_domainParticipant` acts as a *TCP server* listening on port ``5100``
  and is connected to the WAN through a router with public IP ``80.80.99.45``.

* Another :ref:`dds_layer_domainParticipant` acts as a *TCP client* and has configured
  the server's IP address and port in its ``initial_peer`` list.

On the server side, the router must be configured to forward to the *TCP server*
all traffic incomming to port ``5100``. Typically, a NAT routing of port ``5100`` to our
machine is enough. Any existing firewall should be configured as well.

In adition, to allow incoming connections through a WAN,
the :ref:`transport_tcp_v4transportDescriptor` must indicate its **public** IP address
in the ``wan_addr`` data member. The following examples show how to configure
the :ref:`dds_layer_domainParticipant` both in C++ and XML.

+---------------------------------------------------------+
| **C++**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp          |
|    :language: c++                                       |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING-SERVER    |
|    :end-before: //!--                                   |
+---------------------------------------------------------+
| **XML**                                                 |
+---------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml              |
|    :language: xml                                       |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-SERVER |
|    :end-before: <!--><-->                               |
+---------------------------------------------------------+

In the client side, the :ref:`dds_layer_domainParticipant` must be configured
with the **public** IP address and ``listening_port`` of the *TCP server* as
``initial_peer``.

+----------------------------------------------------------+
| **C++**                                                  |
+----------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp           |
|    :language: c++                                        |
|    :start-after: //CONF-TCP-TRANSPORT-SETTING-CLIENT     |
|    :end-before: //!--                                    |
+----------------------------------------------------------+
| **XML**                                                  |
+----------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml               |
|    :language: xml                                        |
|    :start-after: <!-->CONF-TCP-TRANSPORT-SETTING-CLIENT  |
|    :end-before: <!--><-->                                |
+----------------------------------------------------------+


.. _transport_tcp_tls:

TLS over TCP
------------

Fast-RTPS allows configuring a TCP Transport to use TLS (Transport Layer Security).
In order to set up TLS, the ``apply_security`` must be set to ``true`` and the ``tls_config``
must be filled with the desired configuration on the :ref:`transport_tcp_transportDescriptor`.

The following is an example of configuration of TLS on the *TCP server*.

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

The corresponding configuration on the *TCP client* is shown in the following example.

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


The following table describes the data members that are configurable on :class:`TLSConfig`.

.. |TLSVerifyMode| replace:: :ref:`transport_tcp_tls_verifyMode`
.. |TLSOptions| replace:: :ref:`transport_tcp_tls_options`
.. |TLSRole| replace:: :ref:`transport_tcp_tls_role`

+--------------------------+----------------------+-------------+-----------------------------------------------------+
| Member                   | Data type            | Default     | Description                                         |
+==========================+======================+=============+=====================================================+
| ``password``             | ``string``           | empty       | Password of the ``private_key_file`` or             |
|                          |                      |             | ``rsa_private_key_file``.                           |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``private_key_file``     | ``string``           | empty       | Path to the private key certificate file.           |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``rsa_private_key_file`` | ``string``           | empty       | Path to the private key RSA certificate file.       |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``cert_chain_file``      | ``string``           | empty       | Path to the public certificate chain file.          |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``tmp_dh_file``          | ``string``           | empty       | Path to the Diffie-Hellman parameters file.         |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_file``          | ``string``           | empty       | Path to the CA (Certification- Authority) file.     |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_mode``          | ``TLSVerifyMode``    | empty       | Establishes the verification mode mask.             |
|                          |                      |             | See |TLSVerifyMode|                                 |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``options``              | ``TLSOptions``       | empty       | Establishes the SSL Context options mask.           |
|                          |                      |             | See |TLSOptions|                                    |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_paths``         | ``vector<string>``   | empty       | Paths where the system will look for                |
|                          |                      |             | verification files.                                 |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``verify_depth``         | ``int32_t``          | empty       | Maximum allowed depth for verifying                 |
|                          |                      |             | intermediate certificates.                          |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``default_verify_path``  | ``bool``             | empty       | Look for verification files on the default paths.   |
+--------------------------+----------------------+-------------+-----------------------------------------------------+
| ``handshake_role``       | ``TLSHandShakeRole`` | ``DEFAULT`` | Role that the transport will take on handshaking.   |
|                          |                      |             | See |TLSRole|                                       |
+--------------------------+----------------------+-------------+-----------------------------------------------------+

.. _transport_tcp_tls_verifyMode:

TLS Verification Mode
^^^^^^^^^^^^^^^^^^^^^

The verification mode defines how the peer node will be verified.
The following table describes the available verification options.
Several verification options can be combined in the same :ref:`transport_tcp_transportDescriptor`.

+---------------------------------+-----------------------------------------------------------------------------------+
| Value                           | Description                                                                       |
+=================================+===================================================================================+
| ``VERIFY_NONE``                 | Perform no verification.                                                          |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``VERIFY_PEER``                 | Perform verification of the peer.                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``VERIFY_FAIL_IF_NO_PEER_CERT`` | Fail verification if the peer has no certificate.                                 |
|                                 | Ignored unless ``VERIFY_PEER`` is also set.                                       |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``VERIFY_CLIENT_ONCE``          | Do not request client certificate on renegotiation.                               |
|                                 | Ignored unless ``VERIFY_PEER`` is also set.                                       |
+---------------------------------+-----------------------------------------------------------------------------------+


.. _transport_tcp_tls_options:

TLS Options
^^^^^^^^^^^

These options define which TLS features are to be supported.
The following table describes the available options.
Several options can be combined in the same :ref:`transport_tcp_transportDescriptor`.

+---------------------------------+-----------------------------------------------------------------------------------+
| Value                           | Description                                                                       |
+=================================+===================================================================================+
| ``DEFAULT_WORKAROUNDS``         | Perform no verification.                                                          |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_COMPRESSION``              | Disable compression.                                                              |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_SSLV2``                    | Disable SSL v2.                                                                   |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_SSLV3``                    | Disable SSL v3.                                                                   |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1``                    | Disable TLS v1.                                                                   |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1_1``                  | Disable TLS v1.1.                                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1_2``                  | Disable TLS v1.2.                                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``NO_TLSV1_3``                  | Disable TLS v1.3.                                                                 |
+---------------------------------+-----------------------------------------------------------------------------------+
| ``SINGLE_DH_USE``               | Always create a new key when using dh parameters.                                 |
+---------------------------------+-----------------------------------------------------------------------------------+


.. _transport_tcp_tls_role:

TLS Handshake Role
^^^^^^^^^^^^^^^^^^

The role can take the following values:

+---------------------+-----------------------------------------------------------------------------------+
| Value               | Description                                                                       |
+=====================+===================================================================================+
| ``DEFAULT``         | Configured as client if connector, and as server if acceptor                      |
+---------------------+-----------------------------------------------------------------------------------+
| ``CLIENT``          | Configured as client.                                                             |
+---------------------+-----------------------------------------------------------------------------------+
| ``SERVER``          | Configured as server.                                                             |
+---------------------+-----------------------------------------------------------------------------------+


.. _transport_tcp_example:

HelloWorldExampleTCP
--------------------

A TCP version of helloworld example can be found in the ``examples/C++/DDS/HelloWorldExampleTCP`` folder.
It shows a publisher and a subscriber that communicate through TCP.
The publisher is configured as *TCP server* while the Subscriber is acting as *TCP client*.



