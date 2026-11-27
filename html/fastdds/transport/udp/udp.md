<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transport-udp-udp"></a>

<a id="udp-transport"></a>

# UDP Transport

UDP is a connectionless transport, where the receiving [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) must open a UDP port
listening for incoming messages, and the sending DomainParticipant sends messages to this port.

#### WARNING
This documentation assumes the reader has basic knowledge of UDP/IP concepts, since terms like
Time To Live (TTL), socket buffers, and port numbering are not explained in detail.
However, it is possible to configure a basic UDP transport on *Fast DDS* without this knowledge.

<a id="transport-udp-transportdescriptor"></a>

<a id="udptransportdescriptor"></a>

## UDPTransportDescriptor

*eProsima Fast DDS* implements UDP transport for both UDPv4 and UDPv6.
Each of these transports is independent from the other, and has its own [`TransportDescriptorInterface`](../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE).
However, all their [`TransportDescriptorInterface`](../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) data members are common.

The following table describes the common data members for both UDPv4 and UDPv6.

| Member                                                                                                                                                                                                                 | Data type                                                                             | Default      | Description                                                                                                                        |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------|
| [`sendBufferSize`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor14sendBufferSizeE)                                | `uint32_t`                                                                            | 0            | Size of the sending buffer of the socket (octets).                                                                                 |
| [`receiveBufferSize`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor17receiveBufferSizeE)                          | `uint32_t`                                                                            | 0            | Size of the receiving buffer of the socket (octets).                                                                               |
| [`netmask_filter`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor14netmask_filterE)                                | `NetmaskFilterKind`                                                                   | `AUTO`       | See [Netmask filtering](../interfaces.md#netmask-filtering).                                                                       |
| [`allowlist`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_allowlistE)                                | `vector<pair<string, NetmaskFilterKind>>`                                             | Empty vector | List of allowed interfaces with<br/>netmask filter configuration.<br/>See [Allowlist](../interfaces.md#interfaces-allowlist).      |
| [`blocklist`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_blocklistE)                                | `vector<string>`                                                                      | Empty vector | List of blocked interfaces.<br/>See [Blocklist](../interfaces.md#interfaces-blocklist).                                            |
| [`interfaceWhiteList`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor18interfaceWhiteListE)                        | `vector<string>`                                                                      | Empty vector | List of allowed interfaces.<br/>See [Interface Whitelist](../whitelist.md#whitelist-interfaces).                                   |
| [`TTL`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor3TTLE)                                                       | `uint8_t`                                                                             | 1            | Time to live, in number of hops.                                                                                                   |
| [`m_output_udp_socket`](../../api_reference/transport/udp_transport/udp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptor19m_output_udp_socketE)                                           | `uint16_t`                                                                            | 0            | Port number for the outgoing messages.                                                                                             |
| [`non_blocking_send`](../../api_reference/transport/udp_transport/udp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptor17non_blocking_sendE)                                               | `bool`                                                                                | `false`      | Do not block on send operations (\*).                                                                                              |
| [`default_reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor25default_reception_threadsEv) | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |              | Default [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads.           |
| [`reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor17reception_threadsEv)                 | `std::map<uint32_t, ThreadSettings>`                                                  |              | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads on specific ports. |

#### NOTE
When [`non_blocking_send`](../../api_reference/transport/udp_transport/udp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptor17non_blocking_sendE) is set to true, send operations will return immediately if the
buffer is full, but no error will be returned to the upper layer.
This means that the application will behave as if the datagram is sent and lost.
This value is specially useful on high-frequency best-effort writers.

When set to `false`, send operations will block until the network buffer has space for the
datagram.
This may hinder performance on high-frequency writers.

<a id="transport-udp-v4transportdescriptor"></a>

<a id="udpv4transportdescriptor"></a>

### UDPv4TransportDescriptor

[`UDPv4TransportDescriptor`](../../api_reference/transport/udp_transport/udpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24UDPv4TransportDescriptorE) has no additional data members from the common ones described in
[UDPTransportDescriptor](#transport-udp-transportdescriptor).

#### NOTE
The [`kind`](../../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4NK8eprosima7fastdds4rtps18TransportInterface4kindEv) value for a [`UDPv4TransportDescriptor`](../../api_reference/transport/udp_transport/udpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24UDPv4TransportDescriptorE) is given by the value
[`LOCATOR_KIND_UDPv4`](../../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_UDPv4).

<a id="transport-udp-v6transportdescriptor"></a>

<a id="udpv6transportdescriptor"></a>

### UDPv6TransportDescriptor

[`UDPv6TransportDescriptor`](../../api_reference/transport/udp_transport/udpv6_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24UDPv6TransportDescriptorE) has no additional data members from the common ones described in
[UDPTransportDescriptor](#transport-udp-transportdescriptor).

#### NOTE
The [`kind`](../../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4NK8eprosima7fastdds4rtps18TransportInterface4kindEv) value for a [`UDPv6TransportDescriptor`](../../api_reference/transport/udp_transport/udpv6_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24UDPv6TransportDescriptorE) is given by the value
[`LOCATOR_KIND_UDPv6`](../../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_UDPv6).

<a id="transport-udp-enabling"></a>

<a id="enabling-udp-transport"></a>

## Enabling UDP Transport

*Fast DDS* enables a UDPv4 transport by default.
Nevertheless, the application can enable other UDP transports if needed.
To enable a new UDP transport in a [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), first
create an instance of [UDPv4TransportDescriptor](#transport-udp-v4transportdescriptor) (for UDPv4) or
[UDPv6TransportDescriptor](#transport-udp-v6transportdescriptor) (for UDPv6), and add it to the user transport list of the
[DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).

The examples below show this procedure in both C++ code and XML file.

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
udp_transport->sendBufferSize = 9216;
udp_transport->receiveBufferSize = 9216;
udp_transport->non_blocking_send = true;

// [OPTIONAL] ThreadSettings configuration
udp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{2, 2, 2, 2});
udp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{3, 3, 3, 3});

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(udp_transport);

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
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
                <sendBufferSize>9216</sendBufferSize>
                <receiveBufferSize>9216</receiveBufferSize>
                <non_blocking_send>true</non_blocking_send>
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

        <participant profile_name="UDPParticipant">
            <rtps>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
<dds>
```
