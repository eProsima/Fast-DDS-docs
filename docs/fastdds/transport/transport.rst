.. _comm-transports-configuration:

Transports
==========

The transport layer provides communication services between DDS entities,
being responsible of actually sending and receiving messages over a physical transport.
The DDS layer uses this service for both user data and discovery traffic communication.
However, the DDS layer itself is transport independent,
it defines a transport API and can run over any transport plugin that implements this API.
This way, it is not retricted to a specific transport, and applications
can choose the one that best suits their requirements, or create their own.

eProsima Fast DDS comes with five transports already implemented:

- **UDPv4**: UDP Datagram communication over IPv4. This is the default transport created on a new
  :ref:`dds_layer_domainParticipant` if no specific transport configuration is given.

- **UDPv6**: UDP Datagram communication over IPv6.

- **TCPv4**: TCP communication over IPv4.

- **TCPv6**: TCP communication over IPv6.

- **SHM**: Shared memory communication among entities running on the same host.
  See :ref:`transport_sharedMemory_sharedMemory`.

.. toctree::

    /fastdds/transport/transport_api.rst
    /fastdds/transport/shared_memory/shared_memory.rst
    /fastdds/transport/tcp.rst
    /fastdds/transport/listening_locators.rst
    /fastdds/transport/initial_peers.rst
    /fastdds/transport/whitelist.rst
    /fastdds/transport/tips.rst

