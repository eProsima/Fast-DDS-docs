.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _comm-transports-configuration:

Transport Layer
===============

The transport layer provides communication services between DDS entities,
being responsible of actually sending and receiving messages over a physical transport.
The DDS layer uses this service for both user data and discovery traffic communication.
However, the DDS layer itself is transport independent,
it defines a transport API and can run over any transport plugin that implements this API.
This way, it is not restricted to a specific transport, and applications
can choose the one that best suits their requirements, or create their own.

*eProsima Fast DDS* comes with five transports already implemented:

- **UDPv4**: UDP Datagram communication over IPv4.
  This transport is created by default on a new :ref:`dds_layer_domainParticipant`
  if no specific transport configuration is given
  (see :ref:`transport_udp_udp`).

- **UDPv6**: UDP Datagram communication over IPv6
  (see :ref:`transport_udp_udp`).

- **TCPv4**: TCP communication over IPv4
  (see :ref:`transport_tcp_tcp`).

- **TCPv6**: TCP communication over IPv6
  (see :ref:`transport_tcp_tcp`).

- **SHM**: Shared memory communication among entities running on the same host.
  This transport is created by default on a new :ref:`dds_layer_domainParticipant`
  if no specific transport configuration is given
  (see :ref:`transport_sharedMemory_sharedMemory`).

Although it is not part of the transport module,
:ref:`intraprocess data delivery<intraprocess-delivery>` and :ref:`data sharing delivery<datasharing-delivery>`
are also available to send messages between entities on some settings.
The figure below shows a comparison between the different transports available in *Fast DDS*.

.. figure:: /01-figures/fast_dds/transport/transport_comparison.svg
    :align: center


.. toctree::
    :maxdepth: 2

    /fastdds/transport/transport_api.rst
    /fastdds/transport/udp/udp.rst
    /fastdds/transport/tcp/tcp.rst
    /fastdds/transport/shared_memory/shared_memory.rst
    /fastdds/transport/datasharing.rst
    /fastdds/transport/intraprocess.rst
    /fastdds/transport/tcp/tls.rst
    /fastdds/transport/listening_locators.rst
    /fastdds/transport/announced_locators.rst
    /fastdds/transport/whitelist.rst
    /fastdds/transport/disabling_multicast.rst

