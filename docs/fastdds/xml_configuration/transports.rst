.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _transportdescriptors:

Transport descriptors
---------------------

This section defines the XML elements available for configuring the transport layer parameters in *Fast DDS*.
These elements are defined within the XML tag ``<transports_descriptors>``.
The ``<transport_descriptors>`` can contain one or more ``<transport_descriptor>`` XML elements.
Each ``<transport_descriptor>`` element defines a configuration for a specific type of transport protocol.
Each of these ``<transport_descriptor>`` elements are uniquely identified by a transport ID with the ``<transport_id>``
XML tag.
Once the user defines a valid ``<transports_descriptor>``, i.e. defines the transport layer parameters, these
can be loaded
into the XML profile of the DomainParticipant using the ``<transport_id>`` XML tag.
An example of how to load the ``<transport_descriptor>`` into the XML profile of the DomainParticipant is found in
:ref:`participantprofiles`.

The following table lists all the available XML elements that can be defined within the ``<transport_descriptor>``
element for the configuration of the transport layer.
A more detailed explanation of each of these elements can be found in :ref:`comm-transports-configuration`.

+-------------------------------+----------------------------------------------------+----------------------+----------+
| Name                          | Description                                        | Values               | Default  |
+===============================+====================================================+======================+==========+
| ``<transport_id>``            | Unique name to identify each transport             | ``string``           |          |
|                               | descriptor.                                        |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<type>``                    | Type of the transport descriptor.                  | UDPv4                | UDPv4    |
|                               |                                                    +----------------------+          |
|                               |                                                    | UDPv6                |          |
|                               |                                                    +----------------------+          |
|                               |                                                    | TCPv4                |          |
|                               |                                                    +----------------------+          |
|                               |                                                    | TCPv6                |          |
|                               |                                                    +----------------------+          |
|                               |                                                    | SHM                  |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<sendBufferSize>``          | Size in bytes of the send socket buffer. |br|      | ``uint32_t``         | 0        |
|                               | If the value is zero then *Fast DDS* will use |br| |                      |          |
|                               | the system default socket size.                    |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<receiveBufferSize>``       | Size in bytes of the reception socket |br|         | ``uint32_t``         | 0        |
|                               | buffer. If the value is zero then *Fast DDS* |br|  |                      |          |
|                               | will use the system default socket size.           |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<maxMessageSize>``          | The maximum size in bytes of the transport's |br|  | ``uint32_t``         | 65500    |
|                               | message buffer.                                    |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<maxInitialPeersRange>``    | Number of channels opened with each initial |br|   | ``uint32_t``         | 4        |
|                               | remote peer.                                       |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<interfaceWhiteList>``      | Allows defining an interfaces |whitelist|.         | |whitelist|          |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<TTL>``                     | *Time To Live* (**UDP only**). See |br|            | ``uint8_t``          | 1        |
|                               | :ref:`transport_udp_udp`.                          |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<non_blocking_send>``       | Whether to set the non-blocking send mode on |br|  | ``bool``             | ``false``|
|                               | the socket (**UDP only**). See |br|                |                      |          |
|                               | :ref:`transport_udp_transportDescriptor`.          |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<output_port>``             | Port used for output bound. |br|                   | ``uint16_t``         | 0        |
|                               | If this field isn't defined, the output port |br|  |                      |          |
|                               | will be random (**UDP only**).                     |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<wan_addr>``                | Public WAN address when using **TCPv4** |br|       |  IPv4 formatted |br| |          |
|                               | **transports**. This field is optional if the |br| |  ``string``: |br|    |          |
|                               | transport doesn't need to define a WAN |br|        |  ``XXX.XXX.XXX.XXX`` |          |
|                               | address (**TCPv4 only**).                          |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<keep_alive_frequency_ms>`` | Frequency in milliseconds for sending              | ``uint32_t``         | 50000    |
|                               | :ref:`RTCP <rtcpdefinition>` |br|                  |                      |          |
|                               | keep-alive requests (**TCP only**).                |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<keep_alive_timeout_ms>``   | Time in milliseconds since the last |br|           | ``uint32_t``         | 10000    |
|                               | keep-alive request was sent to consider a |br|     |                      |          |
|                               | connection as broken (**TCP only**).               |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<max_logical_port>``        | The maximum number of logical ports to try |br|    | ``uint16_t``         | 100      |
|                               | during :ref:`RTCP<rtcpdefinition>`                 |                      |          |
|                               | negotiations (**TCP only**).                       |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<logical_port_range>``      | The maximum number of logical ports per |br|       | ``uint16_t``         | 20       |
|                               | request to try during                              |                      |          |
|                               | :ref:`RTCP<rtcpdefinition>` negotiations |br|      |                      |          |
|                               | (**TCP only**).                                    |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<logical_port_increment>``  | Increment between logical ports to try during |br| | ``uint16_t``         |  2       |
|                               | :ref:`RTCP<rtcpdefinition>` negotiation |br|       |                      |          |
|                               | (**TCP only**).                                    |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<listening_ports>``         | Local port to work as TCP acceptor for input |br|  | ``List <uint16_t>``  |          |
|                               | connections. If not set, the transport will |br|   |                      |          |
|                               | work as TCP client only (**TCP only**).            |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<tls>``                     | Allows to define TLS related parameters and |br|   | :ref:`tcp-tls`       |          |
|                               | options (**TCP only**).                            |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<calculate_crc>``           | Calculates the Cyclic Redundancy Code (CRC) |br|   | ``bool``             | ``true`` |
|                               | for error control (**TCP only**). |br|             |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<check_crc>``               | Check the CRC for error control (**TCP** |br|      | ``bool``             | ``true`` |
|                               | **only**).                                         |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<enable_tcp_nodelay>``      | Socket option for disabling the Nagle |br|         | ``bool``             | ``false``|
|                               | algorithm. (**TCP only**).                         |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<segment_size>``            | Size (in bytes) of the shared-memory segment. |br| | ``uint32_t``         | 262144   |
|                               | (Optional, **SHM only**).                          |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<port_queue_capacity>``     | Capacity (in number of messages) available to |br| | ``uint32_t``         | 512      |
|                               | every Listener (Optional, **SHM only**).           |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<healthy_check_timeout_ms>``| Maximum time-out (in milliseconds) used when |br|  | ``uint32_t``         | 1000     |
|                               | checking whether a Listener is alive |br|          |                      |          |
|                               | (Optional, **SHM only**).                          |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+
| ``<rtps_dump_file>``          | Complete path (including file) where RTPS |br|     | ``string``           | Empty    |
|                               | messages will be stored for debugging |br|         |                      |          |
|                               | purposes. An empty string indicates no trace |br|  |                      |          |
|                               | will be performed (Optional, **SHM only**).        |                      |          |
+-------------------------------+----------------------------------------------------+----------------------+----------+

The following XML code shows an example of transport protocol configuration using all configurable parameters.
More examples of transports descriptors can be found in the :ref:`comm-transports-configuration` section.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!--XML_PROFILES_TRANSPORT_DESCRIPTORS-->
    :end-before: <!--><-->
    :lines: 2-4, 6-40, 42-43

.. _rtcpdefinition:

.. note::

    The Real-time Transport Control Protocol (`RTCP <https://tools.ietf.org/html/rfc3550>`_) is the control protocol
    for communications with RTPS over TCP/IP connections.

.. _tcp-tls:

TLS Configuration
^^^^^^^^^^^^^^^^^

*Fast DDS* provides mechanisms to configure the Transport Layer Security (TLS) protocol parameters
through the ``<tls>`` XML element of its ``<transport_descriptor>``.
Please, refer to :ref:`transport_tcp_tls` for a detailed explanation of the entire TLS configuration in *Fast DDS*.
More information on how to set up secure communication in *Fast DDS* can be found in the :ref:`security` section.

.. warning::
    For the full understanding of this section, a basic knowledge of network security
    in terms of SSL/TLS, Certificate Authority (CA), Public Key Infrastructure (PKI), and Diffie-Hellman is required;
    encryption protocols are not explained in detail.

The full list of available XML elements that can be defined within the ``<tls>`` element to configure the TLS
protocol are listed in the following table:

.. |DEFconc| replace:: :cpp:concept:`DEFAULT`
.. |VERIFY_FAIL_IF_NO_PEER_CERT| replace:: :cpp:concept:`VERIFY_FAIL_IF_NO_PEER_CERT`

+---------------------------+-----------------------------------+----------------------------------------+-------------+
| Name                      | Description                       | Values                                 | Default     |
+===========================+===================================+========================================+=============+
| ``<password>``            | Password of the                   | ``string``                             |             |
|                           | ``<private_key_file>``            |                                        |             |
|                           | or |br|                           |                                        |             |
|                           | ``<rsa_private_key_file>``        |                                        |             |
|                           | if provided.                      |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<private_key_file>``    | Path to the private key           | ``string``                             |             |
|                           | certificate file.                 |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<rsa_private_key_file>``| Path to the private key           | ``string``                             |             |
|                           | RSA certificate file.             |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<cert_chain_file>``     | Path to the public certificate    | ``string``                             |             |
|                           | chain file.                       |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<tmp_dh_file>``         | Path to the Diffie-Hellman        | ``string``                             |             |
|                           | parameters file                   |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<verify_file>``         | Path to the Certification         | ``string``                             |             |
|                           | Authority (CA) file.              |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<verify_mode>``         | Establishes the verification      | :cpp:concept:`VERIFY_NONE`             |             |
|                           | mode mask. Several |br|           +----------------------------------------+             |
|                           | verification options can be       | :cpp:concept:`VERIFY_PEER`             |             |
|                           | combined in the same |br|         +----------------------------------------+             |
|                           | ``<transport_descriptor>``.       | |VERIFY_FAIL_IF_NO_PEER_CERT|          |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`VERIFY_CLIENT_ONCE`      |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<options>``             | Establishes the SSL Context       | :cpp:concept:`DEFAULT_WORKAROUNDS`     |             |
|                           | options mask. Several |br|        +----------------------------------------+             |
|                           | options can be combined in the    | :cpp:concept:`NO_COMPRESSION`          |             |
|                           | same |br|                         +----------------------------------------+             |
|                           | ``<transport_descriptor>``.       | :cpp:concept:`NO_SSLV2`                |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`NO_SSLV3`                |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`NO_TLSV1`                |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`NO_TLSV1_1`              |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`NO_TLSV1_2`              |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`NO_TLSV1_3`              |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`SINGLE_DH_USE`           |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<verify_paths>``        | Paths where the system will       |  ``string``                            |             |
|                           | look for verification |br| files. |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<verify_depth>``        | Maximum allowed depth to          | ``uint32_t``                           |             |
|                           | verify intermediate |br|          |                                        |             |
|                           | certificates.                     |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<default_verify_path>`` | Specifies whether the system      |  ``bool``                              | ``false``   |
|                           | will look on the |br|             |                                        |             |
|                           | default paths for the             |                                        |             |
|                           | verification files.               |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<handshake_role>``      | Role that the transport will      | :cpp:concept:`DEFAULT`                 | |DEFconc|   |
|                           | take on handshaking. |br|         +----------------------------------------+             |
|                           | On default, the acceptors act     | :cpp:concept:`SERVER`                  |             |
|                           | as :cpp:concept:`SERVER` and the  +----------------------------------------+             |
|                           | |br| connectors as                | :cpp:concept:`CLIENT`                  |             |
|                           | :cpp:concept:`CLIENT`.            |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+

An example of TLS protocol parameter configuration is shown below.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TCP-TLS<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-34, 36-37
