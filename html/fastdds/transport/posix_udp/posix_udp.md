<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transport-posix-udp"></a>

<a id="posix-udpv4-transport-pro"></a>

# POSIX UDPv4 Transport **[Pro]**

The POSIX UDPv4 transport is a UDP transport over IPv4 that only uses POSIX capabilities, while the other UDP and
TCP transports are built on Asio.
Therefore, it can run on platforms that provide nothing beyond POSIX, such as those restricted to a POSIX profile.
It is available on every platform except Windows.

It works like the [UDP Transport](../udp/udp.md#transport-udp-udp): the receiving [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) opens a UDP port
listening for incoming messages, and the sending DomainParticipant sends messages to this port.
Its locators are those of the UDPv4 transport ([`LOCATOR_KIND_UDPv4`](../../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_UDPv4)), so it communicates with DomainParticipants
using the UDPv4 transport, and with any other RTPS implementation over UDPv4.
As both transports handle the same locators, a DomainParticipant uses one of them, not both.

#### WARNING
This documentation assumes the reader has basic knowledge of UDP/IP concepts, since terms like
Time To Live (TTL), socket buffers, and port numbering are not explained in detail.
However, it is possible to configure a basic POSIX UDPv4 transport on *Fast DDS* without this knowledge.

<a id="transport-posix-udp-transportdescriptor"></a>

<a id="posixudpv4transportdescriptor"></a>

## PosixUDPv4TransportDescriptor

[`PosixUDPv4TransportDescriptor`](../../api_reference/transport/udp_transport/posix_udpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps29PosixUDPv4TransportDescriptorE) has the following data members, which work as in the
[UDPTransportDescriptor](../udp/udp.md#transport-udp-transportdescriptor).

| Member                                                                                                                                                                                                                 | Data type                                                                             | Default   | Description                                                                                                                        |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------|-----------|------------------------------------------------------------------------------------------------------------------------------------|
| [`sendBufferSize`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor14sendBufferSizeE)                                | `uint32_t`                                                                            | 0         | Size of the sending buffer of the socket (octets).                                                                                 |
| [`receiveBufferSize`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor17receiveBufferSizeE)                          | `uint32_t`                                                                            | 0         | Size of the receiving buffer of the socket (octets).                                                                               |
| [`TTL`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor3TTLE)                                                       | `uint8_t`                                                                             | 1         | Time to live of the multicast messages, in number of hops.                                                                         |
| [`non_blocking_send`](../../api_reference/transport/udp_transport/posix_udpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps29PosixUDPv4TransportDescriptor17non_blocking_sendE)                                | `bool`                                                                                | `false`   | Do not block on send operations (see [UDPTransportDescriptor](../udp/udp.md#transport-udp-transportdescriptor)).                   |
| [`default_reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor25default_reception_threadsEv) | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |           | Default [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads.           |
| [`reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor17reception_threadsEv)                 | `std::map<uint32_t, ThreadSettings>`                                                  |           | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads on specific ports. |

The following data members of [`SocketTransportDescriptor`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptorE) are not supported.
When they are set, an error is logged and they are ignored.

* [`allowlist`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_allowlistE), [`blocklist`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_blocklistE) and
  [`interfaceWhiteList`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor18interfaceWhiteListE) (see [Interfaces configuration](../interfaces.md#ifaces-config) and [Interface Whitelist](../whitelist.md#whitelist-interfaces)).
* [`netmask_filter`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor14netmask_filterE) set to `ON`
  (see [Netmask filtering](../interfaces.md#netmask-filtering)).

The UDPv4 transport member [`m_output_udp_socket`](../../api_reference/transport/udp_transport/udp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptor19m_output_udp_socketE) does not exist in this transport: the
outgoing messages are sent from ports chosen by the operating system.

#### NOTE
The [`kind`](../../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4NK8eprosima7fastdds4rtps18TransportInterface4kindEv) value for a [`PosixUDPv4TransportDescriptor`](../../api_reference/transport/udp_transport/posix_udpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps29PosixUDPv4TransportDescriptorE) is given by the value
[`LOCATOR_KIND_UDPv4`](../../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_UDPv4).

<a id="transport-posix-udp-interfaces"></a>

<a id="network-interfaces"></a>

## Network interfaces

POSIX has no function that lists the network interfaces of the host, so the transport uses the following addresses
as the interfaces of the host:

* The loopback address, `127.0.0.1`.
* The addresses the operating system sends unicast traffic (through the default route) and multicast traffic from.
* When the host has neither a default nor a multicast route, the addresses its host name resolves to, other than
  loopback ones.
* The addresses of the unicast locators of the DomainParticipant and its endpoints.

These are the addresses announced in the default locators, and the interfaces where the multicast groups are joined
and the multicast messages are sent.
Therefore:

* On a host with several network interfaces, only the one with the default or multicast route is used by default.
  To communicate through the others, add unicast locators with their addresses
  (see [Listening Locators](../listening_locators.md#listening-locators)).
* On a network without a default nor a multicast route, where the host name does not resolve to an address of the
  host (e.g. it resolves to `127.0.1.1`), only the loopback is found, and the DomainParticipant can only communicate
  with others on the same host.
  To communicate with other hosts, add unicast locators with an address of the host.
* The changes of the network interfaces are not detected.

<a id="transport-posix-udp-enabling"></a>

<a id="enabling-posix-udpv4-transport"></a>

## Enabling POSIX UDPv4 Transport

To enable a POSIX UDPv4 transport in a [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), first
create an instance of [PosixUDPv4TransportDescriptor](#transport-posix-udp-transportdescriptor), and add it to the user transport list of the
[DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
As it replaces the UDPv4 transport, which *Fast DDS* enables by default, the builtin transports must be disabled.

The examples below show this procedure in both C++ code and XML file.

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
auto posix_udp_transport = std::make_shared<PosixUDPv4TransportDescriptor>();
posix_udp_transport->sendBufferSize = 9216;
posix_udp_transport->receiveBufferSize = 9216;

// [OPTIONAL] ThreadSettings configuration
posix_udp_transport->default_reception_threads(ThreadSettings{2, 2, 2, 2});
posix_udp_transport->set_thread_config_for_port(12345, ThreadSettings{3, 3, 3, 3});

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(posix_udp_transport);

// Avoid using the default transport
qos.transport().use_builtin_transports = false;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>posix_udp_transport</transport_id>
                <type>POSIX_UDPv4</type>
                <sendBufferSize>9216</sendBufferSize>
                <receiveBufferSize>9216</receiveBufferSize>
                <default_reception_threads>
                    <scheduling_policy>2</scheduling_policy>
                    <priority>2</priority>
                    <affinity>2</affinity>
                    <stack_size>2</stack_size>
                </default_reception_threads>
                <reception_threads>
                    <reception_thread port="12345">
                        <scheduling_policy>3</scheduling_policy>
                        <priority>3</priority>
                        <affinity>3</affinity>
                        <stack_size>3</stack_size>
                    </reception_thread>
                </reception_threads>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="PosixUDPParticipant">
            <rtps>
                <userTransports>
                    <transport_id>posix_udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="transport-posix-udp-builtin"></a>

<a id="building-without-the-other-udp-and-tcp-transports"></a>

## Building without the other UDP and TCP transports

When *Fast DDS* is built with the CMake option `FASTDDS_AVOID_ASIO` set to `ON` (see [CMake options](../../../installation/configuration/cmake_options.md#cmake-options)),
the UDPv4, UDPv6, TCPv4 and TCPv6 transports are left out, and the POSIX UDPv4 transport takes the place of the UDPv4
one:

* The builtin transports use a POSIX UDPv4 transport instead of the UDPv4 one.
* The builtin transports configurations that use the UDPv6 or TCP transports (`DEFAULTv6`, `UDPv6`,
  `LARGE_DATA`, `LARGE_DATAv6` and `P2P`) are not available.
* The XML transport types `UDPv4`, `UDPv6`, `TCPv4` and `TCPv6` are not available: use `POSIX_UDPv4`.
