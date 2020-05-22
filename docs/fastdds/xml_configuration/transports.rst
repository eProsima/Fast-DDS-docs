.. _transportdescriptors:

Transport descriptors
---------------------

This section allows creating transport descriptors to be referenced by the :ref:`participantprofiles`.
Once a well-defined transport descriptor is referenced by a **Participant profile**, every time that profile
is instantiated it will use or create the related transport.

The following XML code shows the complete list of configurable parameters:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-TRANSPORT-DESCRIPTORS<-->
    :lines: 1, 11-39, 53

The XML label ``<transport_descriptors>`` can hold any number of ``<transport_descriptor>``.

+-------------------------------+-----------------------------------+---------------------------------+----------------+
| Name                          | Description                       | Values                          | Default        |
+===============================+===================================+=================================+================+
| ``<transport_id>``            | Unique name to identify each      | ``string``                      |                |
|                               | transport descriptor.             |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<type>``                    | Type of the transport descriptor. | :class:`UDPv4`, :class:`UDPv6`, | :class:`UDPv4` |
|                               |                                   | :class:`TCPv4`, :class:`TCPv6`, |                |
|                               |                                   | :class:`SHM`                    |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<sendBufferSize>``          | Size in bytes of the socket       | ``uint32``                      | 0              |
|                               | send buffer.                      |                                 |                |
|                               | If the value is zero then         |                                 |                |
|                               | FastRTPS will use the default     |                                 |                |
|                               | size from the configuration of    |                                 |                |
|                               | the sockets, using a minimum      |                                 |                |
|                               | size of 65536 bytes.              |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<receiveBufferSize>``       | Size in bytes of the socket       | ``uint32``                      | 0              |
|                               | receive buffer.                   |                                 |                |
|                               | If the value is zero then         |                                 |                |
|                               | FastRTPS will use the default     |                                 |                |
|                               | size from the configuration of    |                                 |                |
|                               | the sockets, using a minimum      |                                 |                |
|                               | size of 65536 bytes.              |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<TTL>``                     | *Time To Live*, **only**          | ``uint8``                       | 1              |
|                               | for UDP transports .              |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<non_blocking_send>``       | Whether to set the non-blocking   | ``bool``                        | false          |
|                               | send mode on the socket           |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<maxMessageSize>``          | The maximum size in bytes         | ``uint32``                      | 65500          |
|                               | of the transport's message        |                                 |                |
|                               | buffer.                           |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<maxInitialPeersRange>``    | The maximum number of             | ``uint32``                      | 4              |
|                               | guessed initial peers             |                                 |                |
|                               | to try to connect.                |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<interfaceWhiteList>``      | Allows defining                   | :ref:`whitelist-interfaces`     |                |
|                               | :ref:`whitelist-interfaces`.      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<wan_addr>``                | Public WAN address when           |  ``string`` with IPv4 Format    |                |
|                               | using **TCPv4 transports**.       |                                 |                |
|                               | This field is optional if         |  :class:`XXX.XXX.XXX.XXX`.      |                |
|                               | the transport doesn't need        |                                 |                |
|                               | to define a WAN address.          |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<output_port>``             | Port used for output bound.       | ``uint16``                      | 0              |
|                               | If this field isn't defined,      |                                 |                |
|                               | the output port will be random    |                                 |                |
|                               | (UDP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<keep_alive_frequency_ms>`` | Frequency in milliseconds         | ``uint32``                      | 50000          |
|                               | for sending                       |                                 |                |
|                               | :ref:`RTCP<rtcpdefinition>`       |                                 |                |
|                               | keep-alive                        |                                 |                |
|                               | requests (TCP **only**).          |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<keep_alive_timeout_ms>``   | Time in milliseconds since        | ``uint32``                      | 10000          |
|                               | sending the last keep-alive       |                                 |                |
|                               | request to consider a connection  |                                 |                |
|                               | as broken. (TCP **only**).        |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<max_logical_port>``        | The maximum number of logical     | ``uint16``                      | 100            |
|                               | ports to try during               |                                 |                |
|                               | :ref:`RTCP<rtcpdefinition>`       |                                 |                |
|                               | negotiations. (TCP **only**)      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<logical_port_range>``      | The maximum number of logical     | ``uint16``                      | 20             |
|                               | ports per request to try          |                                 |                |
|                               | during :ref:`RTCP<rtcpdefinition>`|                                 |                |
|                               | negotiations. (TCP **only**)      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<logical_port_increment>``  | Increment between logical         | ``uint16``                      |  2             |
|                               | ports to try during               |                                 |                |
|                               | :ref:`RTCP<rtcpdefinition>`       |                                 |                |
|                               | negotiation. (TCP **only**)       |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<listening_ports>``         | Local port to work as TCP         | ``List <uint16>``               |                |
|                               | acceptor for input connections.   |                                 |                |
|                               | If not set, the transport will    |                                 |                |
|                               | work as TCP client only           |                                 |                |
|                               | (TCP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<tls>``                     | Allows to define TLS related      | :ref:`tcp-tls`                  |                |
|                               | parameters and options            |                                 |                |
|                               | (TCP **only**).                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<segment_size>``            | Size (in bytes) of the            | ``uint32``                      | 262144         |
|                               | shared-memory segment.            |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<port_queue_capacity>``     | Capacity (in number of messages)  | ``uint32``                      | 512            |
|                               | available to every Listener       |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<healthy_check_timeout_ms>``| Maximum time-out (in milliseconds)| ``uint32``                      | 1000           |
|                               | used when checking whether a      |                                 |                |
|                               | Listener is alive & OK            |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<rtps_dump_file>``          | Complete path (including file)    | ``string``                      | empty          |
|                               | where RTPS messages will be       |                                 |                |
|                               | stored for debugging purposes.    |                                 |                |
|                               | An empty string indicates no      |                                 |                |
|                               | trace will be performed           |                                 |                |
|                               | (OPTIONAL, SHM **only**).         |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+



.. _rtcpdefinition:

RTCP is the control protocol for communications with RTPS over TCP/IP connections.

There are more examples of transports descriptors in :ref:`comm-transports-configuration`.

.. _tcp-tls:

TLS Configuration
^^^^^^^^^^^^^^^^^

Fast-RTPS allows configuring TLS parameters through the ``<tls>`` tag of its Transport Descriptor.
The full list of options is listed here:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TCP-TLS<-->
    :end-before: <!--><-->

+-------------------------------+-----------------------------------+---------------------------------+----------------+
| Name                          | Description                       | Values                          | Default        |
+===============================+===================================+=================================+================+
| ``<password>``                | Password of the private_key_file  | ``string``                      |                |
|                               | if provided (or RSA).             |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<private_key_file>``        | Path to the private key           | ``string``                      |                |
|                               | certificate file.                 |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<rsa_private_key_file>``    | Path to the private key           | ``string``                      |                |
|                               | RSA certificate file.             |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<cert_chain_file>``         | Path to the public certificate    | ``string``                      |                |
|                               | chain file.                       |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<tmp_dh_file>``             | Path to the Diffie-Hellman        | ``string``                      |                |
|                               | parameters file                   |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_file>``             | Path to the CA (Certification-    | ``string``                      |                |
|                               | Authority) file.                  |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_mode>``             | Establishes the verification      | ``VERIFY_NONE``,                |                |
|                               | mode mask.                        | ``VERIFY_PEER``,                |                |
|                               |                                   | ``VERIFY_FAIL_IF_NO_PEER_CERT``,|                |
|                               |                                   | ``VERIFY_CLIENT_ONCE``          |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<options>``                 | Establishes the SSL Context       | ``DEFAULT_WORKAROUNDS``,        |                |
|                               | options mask                      | ``NO_COMPRESSION``,             |                |
|                               |                                   | ``NO_SSLV2``,                   |                |
|                               |                                   | ``NO_SSLV3``,                   |                |
|                               |                                   | ``NO_TLSV1``,                   |                |
|                               |                                   | ``NO_TLSV1_1``,                 |                |
|                               |                                   | ``NO_TLSV1_2``,                 |                |
|                               |                                   | ``NO_TLSV1_3``,                 |                |
|                               |                                   | ``SINGLE_DH_USE``               |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_paths>``            | Paths where the system will       |  ``string``                     |                |
|                               | look for verification files.      |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<verify_depth>``            | Maximum allowed depth for         | ``uint32``                      |                |
|                               | verify intermediate certificates. |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<default_verify_path>``     | Default paths where the system    |  ``boolean``                    | ``false``      |
|                               | will look for verification files. |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
| ``<handshake_role>``          | Role that the transport will      | ``DEFAULT``,                    | ``DEFAULT``    |
|                               | take on handshaking.              | ``SERVER``,                     |                |
|                               | On default, the acceptors act as  | ``CLIENT``                      |                |
|                               | ``SERVER`` and the connectors as  |                                 |                |
|                               | ``CLIENT``.                       |                                 |                |
+-------------------------------+-----------------------------------+---------------------------------+----------------+
