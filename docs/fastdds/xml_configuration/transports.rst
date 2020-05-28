.. include:: includes/aliases.rst

.. _transportdescriptors:

Transport descriptors
---------------------

This section defines the XML elements available for configuring the transport layer parameters in Fast DDS.
These elements are defined within the XML tag ``<transports_descriptors>``.
The ``<transport_descriptors>`` can contain one or more ``<transport_descriptor>`` XML elements.
Each ``<transport_descriptor>`` element builds a configuration for a specific type of transport protocol.
Each of these ``<transport_descriptor>`` elements are uniquely identified by a transport ID with the ``<transport_id>``
XML tag.
Once the user defines a valid ``<transports_descriptor>``, i.e. defines the transport layer parameters, these
can be loaded
into the XML profile of the DomainParticipant using the ``<transport_id>`` XML tag.
An example of how to load the ``<transport_descriptor>`` into the XML profile of the DomainParticipant is found in
the :ref:`participantprofiles` section.

The following table lists all the available XML elements which can be defined within the ``<transport_descriptor>``
element for the configuration of the transport protocol parameters.

+-------------------------------+----------------------------------------------------+-----------------------+---------+
| Name                          | Description                                        | Values                | Default |
+===============================+====================================================+=======================+=========+
| ``<transport_id>``            | Unique name to identify each transport             | ``string``            |         |
|                               | descriptor.                                        |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<type>``                    | Type of the transport descriptor.                  | UDPv4                 | UDPv4   |
|                               |                                                    +-----------------------+         |
|                               |                                                    | UDPv6                 |         |
|                               |                                                    +-----------------------+         |
|                               |                                                    | TCPv4                 |         |
|                               |                                                    +-----------------------+         |
|                               |                                                    | TCPv6                 |         |
|                               |                                                    +-----------------------+         |
|                               |                                                    | SHM                   |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<sendBufferSize>``          | Size in bytes of the socket send buffer. |br|      | ``uint32``            | 0       |
|                               | If the value is zero then Fast DDS will use |br|   |                       |         |
|                               | the default size from the sockets |br|             |                       |         |
|                               | configuration ,which has a minimum size of |br|    |                       |         |
|                               | 65536 bytes.                                       |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<receiveBufferSize>``       | Size in bytes of the socket receive buffer. |br|   | ``uint32``            | 0       |
|                               | If the value is zero then Fast DDS will use |br|   |                       |         |
|                               | the default size from the sockets |br|             |                       |         |
|                               | configuration ,which has a minimum size of |br|    |                       |         |
|                               | 65536 bytes.                                       |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<TTL>``                     | *Time To Live* (UDP **only**).                     | ``uint8``             | 1       |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<non_blocking_send>``       | Whether to set the non-blocking send mode on |br|  | ``bool``              | false   |
|                               | the socket                                         |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<maxMessageSize>``          | The maximum size in bytes of the transport's |br|  | ``uint32``            | 65500   |
|                               | message buffer.                                    |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<maxInitialPeersRange>``    | The maximum number of guessed initial peers |br|   | ``uint32``            | 4       |
|                               | to try to connect.                                 |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<interfaceWhiteList>``      | Allows defining a |whitelist| interfaces.          | |whitelist|           |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<wan_addr>``                | Public WAN address when using **TCPv4** |br|       |  IPv4 formatted |br|  |         |
|                               | **transports**. This field is optional if the |br| |  ``string``: |br|     |         |
|                               | transport doesn't need to define a WAN address.    |  ``XXX.XXX.XXX.XXX``. |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<output_port>``             | Port used for output bound. |br|                   | ``uint16``            | 0       |
|                               | If this field isn't defined, the output port |br|  |                       |         |
|                               | will be random (UDP **only**).                     |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<keep_alive_frequency_ms>`` | Frequency in milliseconds for sending              | ``uint32``            | 50000   |
|                               | :ref:`RTCP <rtcpdefinition>` |br|                  |                       |         |
|                               | keep-alive requests (TCP **only**).                |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<keep_alive_timeout_ms>``   | Time in milliseconds since the last |br|           | ``uint32``            | 10000   |
|                               | keep-alive request was sent to consider a |br|     |                       |         |
|                               | connection as broken (TCP **only**).               |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<max_logical_port>``        | The maximum number of logical ports to try |br|    | ``uint16``            | 100     |
|                               | during :ref:`RTCP<rtcpdefinition>`                 |                       |         |
|                               | negotiations (TCP **only**).                       |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<logical_port_range>``      | The maximum number of logical ports per |br|       | ``uint16``            | 20      |
|                               | request to try during                              |                       |         |
|                               | :ref:`RTCP<rtcpdefinition>` negotiations |br|      |                       |         |
|                               | (TCP **only**).                                    |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<logical_port_increment>``  | Increment between logical ports to try during |br| | ``uint16``            |  2      |
|                               | ports to try during                                |                       |         |
|                               | :ref:`RTCP<rtcpdefinition>` negotiation |br|       |                       |         |
|                               | (TCP **only**).                                    |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<listening_ports>``         | Local port to work as TCP acceptor for input |br|  | ``List <uint16>``     |         |
|                               | connections. If not set, the transport will |br|   |                       |         |
|                               | work as TCP client only (TCP **only**).            |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<tls>``                     | Allows to define TLS related parameters and |br|   | :ref:`tcp-tls`        |         |
|                               | options (TCP **only**).                            |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<segment_size>``            | Size (in bytes) of the shared-memory segment. |br| | ``uint32``            | 262144  |
|                               | (Optional, SHM **only**).                          |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<port_queue_capacity>``     | Capacity (in number of messages) available to |br| | ``uint32``            | 512     |
|                               | every Listener (Optional, SHM **only**). |br|      |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<healthy_check_timeout_ms>``| Maximum time-out (in milliseconds) used when |br|  | ``uint32``            | 1000    |
|                               | checking whether a Listener is alive |br|          |                       |         |
|                               | (Optional, SHM **only**).                          |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+
| ``<rtps_dump_file>``          | Complete path (including file) where RTPS |br|     | ``string``            | empty   |
|                               | messages will be stored for debugging |br|         |                       |         |
|                               | purposes. An empty string indicates no trace |br|  |                       |         |
|                               | will be performed (Optional, SHM **only**).        |                       |         |
+-------------------------------+----------------------------------------------------+-----------------------+---------+

The following XML code shows an example of transport protocol configuration using all configurable parameters.
More examples of transports descriptors can be found in the :ref:`comm-transports-configuration` section.


.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!--XML_PROFILES_TRANSPORT_DESCRIPTORS-->
    :end-before: <!--><-->

.. _rtcpdefinition:

.. note::

    The Real-time Transport Control Protocol (RTCP) is the control protocol for communications with RTPS over
    TCP/IP connections.

.. _tcp-tls:

TLS Configuration
^^^^^^^^^^^^^^^^^

Fast DDS provides the mechanisms to configure the Transport Layer Security (TLS) protocol parameters
through the ``<tls>`` XML element of its ``<transport_descriptor>``.
More information on how to set up secure communication in Fast DDS can be found in the :ref:`security` section.

.. warning::
    For the full understanding of this section it is required the user to have basic knowledge of network security
    since terms like SSL/TLS, Certificate Authority (CA), Public Key Infrastructure (PKI), and Diffie-Hellman
    encryption protocol are not explained in detail.
    However, it is possible to configure basic TLS security parameters using the code snippet shown below.

The full list of available XML elements, which can be defined within the ``<tls>`` element to configure the TLS
protocol, are listed in the following table:

.. |DEFconc| replace:: :cpp:concept:`DEFAULT`
.. |VERIFY_FAIL_IF_NO_PEER_CERT| replace:: :cpp:concept:`VERIFY_FAIL_IF_NO_PEER_CERT`

+---------------------------+-----------------------------------+----------------------------------------+-------------+
| Name                      | Description                       | Values                                 | Default     |
+===========================+===================================+========================================+=============+
| ``<password>``            | Password of the private_key_file  | ``string``                             |             |
|                           | if provided |br| (or RSA).        |                                        |             |
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
| ``<verify_file>``         | Path to the Certification-        | ``string``                             |             |
|                           | Authority (CA) file.              |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<verify_mode>``         | Establishes the verification      | :cpp:concept:`VERIFY_NONE`             |             |
|                           | mode mask.                        +----------------------------------------+             |
|                           |                                   | :cpp:concept:`VERIFY_PEER`             |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | |VERIFY_FAIL_IF_NO_PEER_CERT|          |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`VERIFY_CLIENT_ONCE`      |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<options>``             | Establishes the SSL Context       | :cpp:concept:`DEFAULT_WORKAROUNDS`     |             |
|                           | options mask.                     +----------------------------------------+             |
|                           |                                   | :cpp:concept:`NO_COMPRESSION`          |             |
|                           |                                   +----------------------------------------+             |
|                           |                                   | :cpp:concept:`NO_SSLV2`                |             |
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
| ``<verify_depth>``        | Maximum allowed depth for         | ``uint32``                             |             |
|                           | verify intermediate |br|          |                                        |             |
|                           | certificates.                     |                                        |             |
+---------------------------+-----------------------------------+----------------------------------------+-------------+
| ``<default_verify_path>`` | Default paths where the system    |  ``boolean``                           | ``false``   |
|                           | will look for |br| verification   |                                        |             |
|                           | files.                            |                                        |             |
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
