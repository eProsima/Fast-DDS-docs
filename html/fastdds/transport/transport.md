<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="comm-transports-configuration"></a>

<a id="transport-layer"></a>

# Transport Layer

The transport layer provides communication services between DDS entities,
being responsible of actually sending and receiving messages over a physical transport.
The DDS layer uses this service for both user data and discovery traffic communication.
However, the DDS layer itself is transport independent,
it defines a transport API and can run over any transport plugin that implements this API.
This way, it is not restricted to a specific transport, and applications
can choose the one that best suits their requirements, or create their own.

*eProsima Fast DDS* comes with five transports already implemented:

- **UDPv4**: UDP Datagram communication over IPv4.
  This transport is created by default on a new [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
  if no specific transport configuration is given
  (see [UDP Transport](udp/udp.md#transport-udp-udp)).
- **UDPv6**: UDP Datagram communication over IPv6
  (see [UDP Transport](udp/udp.md#transport-udp-udp)).
- **TCPv4**: TCP communication over IPv4
  (see [TCP Transport](tcp/tcp.md#transport-tcp-tcp)).
- **TCPv6**: TCP communication over IPv6
  (see [TCP Transport](tcp/tcp.md#transport-tcp-tcp)).
- **SHM**: Shared memory communication among entities running on the same host.
  This transport is created by default on a new [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
  if no specific transport configuration is given
  (see [Shared Memory Transport](shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory)).

Although it is not part of the transport module,
[intraprocess data delivery](intraprocess.md#intraprocess-delivery) and [data sharing delivery](datasharing.md#datasharing-delivery)
are also available to send messages between entities on some settings.
The figure below shows a comparison between the different transports available in *Fast DDS*.

![image](01-figures/fast_dds/transport/transport_comparison.svg)

* [Transport API](transport_api.md)
  * [TransportDescriptorInterface](transport_api.md#transportdescriptorinterface)
  * [TransportInterface](transport_api.md#transportinterface)
  * [Locator](transport_api.md#locator)
  * [Chaining of transports](transport_api.md#chaining-of-transports)
* [UDP Transport](udp/udp.md)
  * [UDPTransportDescriptor](udp/udp.md#udptransportdescriptor)
  * [Enabling UDP Transport](udp/udp.md#enabling-udp-transport)
* [TCP Transport](tcp/tcp.md)
  * [TCPTransportDescriptor](tcp/tcp.md#tcptransportdescriptor)
  * [Enabling TCP Transport](tcp/tcp.md#enabling-tcp-transport)
  * [TCPv6 Addresses and Interfaces](tcp/tcp.md#tcpv6-addresses-and-interfaces)
  * [WAN or Internet Communication over TCPv4](tcp/tcp.md#wan-or-internet-communication-over-tcpv4)
  * [WAN or Internet Communication over TCPv6](tcp/tcp.md#wan-or-internet-communication-over-tcpv6)
  * [Delivery Mechanisms example](tcp/tcp.md#delivery-mechanisms-example)
* [Shared Memory Transport](shared_memory/shared_memory.md)
  * [Definition of Concepts](shared_memory/shared_memory.md#definition-of-concepts)
  * [SharedMemTransportDescriptor](shared_memory/shared_memory.md#sharedmemtransportdescriptor)
  * [Enabling Shared Memory Transport](shared_memory/shared_memory.md#enabling-shared-memory-transport)
  * [Delivery Mechanisms example](shared_memory/shared_memory.md#delivery-mechanisms-example)
* [Data-sharing delivery](datasharing.md)
  * [Overview](datasharing.md#overview)
  * [Constraints](datasharing.md#constraints)
  * [Data-sharing delivery configuration](datasharing.md#data-sharing-delivery-configuration)
  * [DataReader and DataWriter history coupling](datasharing.md#datareader-and-datawriter-history-coupling)
* [Intra-process delivery](intraprocess.md)
  * [GUID Prefix considerations for intra-process delivery](intraprocess.md#guid-prefix-considerations-for-intra-process-delivery)
* [TLS over TCP](tcp/tls.md)
  * [TLS Verification Mode](tcp/tls.md#tls-verification-mode)
  * [TLS Options](tcp/tls.md#tls-options)
  * [TLS Handshake Role](tcp/tls.md#tls-handshake-role)
* [Low Bandwidth Transports <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](low-bandwidth-transports.md)
  * [Payload Compression Transport](low-bandwidth-transports.md#payload-compression-transport)
  * [Header Reduction Transport](low-bandwidth-transports.md#header-reduction-transport)
  * [Full example](low-bandwidth-transports.md#full-example)
* [Listening Locators](listening_locators.md)
  * [Adding Listening Locators](listening_locators.md#adding-listening-locators)
  * [Default Listening Locators](listening_locators.md#default-listening-locators)
  * [Well Known Ports](listening_locators.md#well-known-ports)
* [Announced Locators](announced_locators.md)
  * [Default Announced Locators](announced_locators.md#default-announced-locators)
  * [External Locators](announced_locators.md#external-locators)
* [Interface Whitelist](whitelist.md)
* [Interfaces configuration](interfaces.md)
  * [Netmask filtering](interfaces.md#netmask-filtering)
  * [Allowlist](interfaces.md#allowlist)
  * [Blocklist](interfaces.md#blocklist)
* [Disabling all Multicast Traffic](disabling_multicast.md)
