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

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<transport_id>``
     - Unique name to identify each transport descriptor.
     - ``string``
     -
   * - ``<type>``
     - Type of the transport descriptor.
     - ``UDPv4``, ``UDPv6``, ``TCPv4``, ``TCPv6``, ``SHM``
     - ``UDPv4``
   * - ``<sendBufferSize>``
     - Size in bytes of the send socket buffer. If the value is zero, *Fast DDS* will use the system default socket size.
     - ``uint32_t``
     - ``0``
   * - ``<receiveBufferSize>``
     - Size in bytes of the reception socket buffer. If the value is zero, *Fast DDS* will use the system default socket size.
     - ``uint32_t``
     - ``0``
   * - ``<maxMessageSize>``
     - The maximum size in bytes of the transport's message buffer.
     - ``uint32_t``
     - ``65500``
   * - ``<maxInitialPeersRange>``
     - Number of channels opened with each initial remote peer.
     - ``uint32_t``
     - ``4``
   * - ``<netmask_filter>``
     - Transport's :ref:`netmask_filtering` configuration.
     - ``string``
     - ``AUTO``
   * - ``<interfaces>``
     - Allows defining an :ref:`ifaces_config`.
     - ``string``
     -
   * - ``<interfaceWhiteList>``
     - Allows defining an interfaces |whitelist|.
     - ``string``
     -
   * - ``<TTL>``
     - Time To Live (UDP only). See  :ref:`transport_udp_udp`.
     - ``uint8_t``
     - ``1``
   * - ``<non_blocking_send>``
     - Whether to set the non-blocking send mode on the socket (NOT available for SHM type). See :ref:`transport_udp_transportDescriptor` and :ref:`transport_tcp_transportDescriptor`.
     - ``bool``
     - ``false``
   * - ``<output_port>``
     - Port used for output bound. If this field isn't defined, the output port will be random (UDP only).
     - ``uint16_t``
     - ``0``
   * - ``<wan_addr>``
     - Public WAN address when using TCPv4 transports. This field is optional if the transport doesn't need to define a WAN address (TCPv4 only).
     - ``string``
     -
   * - ``<keep_alive_frequency_ms>``
     - Frequency in milliseconds for sending :ref:`RTCP <rtcpdefinition>` keep-alive requests (TCP only).
     - ``uint32_t``
     - ``50000``
   * - ``<keep_alive_timeout_ms>``
     - Time in milliseconds since the last keep-alive request was sent to consider a connection as broken (TCP only).
     - ``uint32_t``
     - ``10000``
   * - ``<max_logical_port>``
     - The maximum number of logical ports to try during :ref:`RTCP <rtcpdefinition>` negotiations (TCP only).
     - ``uint16_t``
     - ``100``
   * - ``<logical_port_range>``
     - The maximum number of logical ports per request to try during :ref:`RTCP <rtcpdefinition>` negotiations (TCP only).
     - ``uint16_t``
     - ``20``
   * - ``<logical_port_increment>``
     - Increment between logical ports to try during :ref:`RTCP <rtcpdefinition>` negotiation (TCP only).
     - ``uint16_t``
     - ``2``
   * - ``<listening_ports>``
     - Local port to work as TCP acceptor for input connections. If not set, the transport will work as TCP client only. If set to 0, an available port will be automatically assigned (TCP only).
     - ``List<uint16_t>``
     -
   * - ``<tls>``
     - Allows to define TLS related parameters and options (TCP only).
     - :ref:`tcp-tls`
     -
   * - ``<calculate_crc>``
     - Calculates the Cyclic Redundancy Code (CRC) for error control (TCP only).
     - ``bool``
     - ``true``
   * - ``<check_crc>``
     - Check the CRC for error control (TCP only).
     - ``bool``
     - ``true``
   * - ``<enable_tcp_nodelay>``
     - Socket option for disabling the Nagle algorithm (TCP only).
     - ``bool``
     - ``false``
   * - ``<tcp_negotiation_timeout>``
     - Time to wait for logical port negotiation (in ms) (TCP only).
     - ``uint32_t``
     - ``0``
   * - ``<keep_alive_thread>``
     - Settings for the keep-alive thread.
     - |ThreadSettings|
     -
   * - ``<accept_thread>``
     - Settings for the accept thread.
     - |ThreadSettings|
     -
   * - ``<segment_size>``
     - Size (in bytes) of the shared-memory segment (Optional, SHM only).
     - ``uint32_t``
     - ``262144``
   * - ``<port_queue_capacity>``
     - Capacity (in number of messages) available to every Listener (Optional, SHM only).
     - ``uint32_t``
     - ``512``
   * - ``<healthy_check_timeout_ms>``
     - Maximum time-out (in milliseconds) used when checking whether a Listener is alive (Optional, SHM only).
     - ``uint32_t``
     - ``1000``
   * - ``<rtps_dump_file>``
     - Complete path (including file) where RTPS messages will be stored for debugging purposes. An empty string indicates no trace will be performed (Optional, SHM only).
     - ``string``
     - ``Empty``
   * - ``<default_reception_threads>``
     - Default |ThreadSettings| for the reception threads.
     - |ThreadSettings|
     -
   * - ``<reception_threads>``
     - |ThreadSettings| for the reception threads on specific ports.
     - ``std::map<uint32_t, ThreadSettings>``
     -
   * - ``<dump_thread>``
     - |ThreadSettings| for the SHM dump thread.
     - |ThreadSettings|
     -

The following XML code shows an example of transport protocol configuration using all configurable parameters.
More examples of transports descriptors can be found in the :ref:`comm-transports-configuration` section.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!--XML_PROFILES_TRANSPORT_DESCRIPTORS-->
    :end-before: <!--><-->
    :lines: 2-4, 6-70, 72-73

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
|                           | or                                |                                        |             |
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
|                           | mode mask. Several                +----------------------------------------+             |
|                           | verification options can be       | :cpp:concept:`VERIFY_PEER`             |             |
|                           | combined in the same              +----------------------------------------+             |
|                           | ``<transport_descriptor>``.       | |VERIFY_FAIL_IF_NO_PEER_CERT|          |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`VERIFY_CLIENT_ONCE`      |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<options>``             | Establishes the SSL Context       | :cpp:concept:`DEFAULT_WORKAROUNDS`     |             |
|                           | options mask. Several             +----------------------------------------+             |
|                           | options can be combined in the    | :cpp:concept:`NO_COMPRESSION`          |             |
|                           | same                              +----------------------------------------+             |
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
| ``<verify_paths>``        | Paths where the system will       |  ``List <string>``                     |             |
|                           | look for verification      files. |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<verify_depth>``        | Maximum allowed depth to          | ``uint32_t``                           |             |
|                           | verify intermediate               |                                        |             |
|                           | certificates.                     |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<default_verify_path>`` | Specifies whether the system      |  ``bool``                              | ``false``   |
|                           | will look on the                  |                                        |             |
|                           | default paths for the             |                                        |             |
|                           | verification files.               |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<handshake_role>``      | Role that the transport will      | :cpp:concept:`DEFAULT`                 | |DEFconc|   |
|                           | take on handshaking.              +----------------------------------------+             |
|                           | On default, the acceptors act     | :cpp:concept:`SERVER`                  |             |
|                           | as :cpp:concept:`SERVER` and the  +----------------------------------------+             |
|                           | connectors as                     | :cpp:concept:`CLIENT`                  |             |
|                           | :cpp:concept:`CLIENT`.            |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<server_name>``         | server name or host name required | ``string``                             |             |
|                           | in case Server Name Indication    |                                        |             |
|                           | (SNI) is used.                    |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+

An example of TLS protocol parameter configuration is shown below.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TCP-TLS<-->
    :end-before: <!--><-->
    :lines: 2-4, 6-34, 36-37
