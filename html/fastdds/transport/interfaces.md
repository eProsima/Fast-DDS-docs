<!-- {{{ X-Types API -->
<!-- }}} -->

#### WARNING
The configuration options described in this section apply to network interfaces.
Therefore, it is only available on [TCP Transport](tcp/tcp.md#transport-tcp-tcp) and [UDP Transport](udp/udp.md#transport-udp-udp).

<a id="ifaces-config"></a>

<a id="interfaces-configuration"></a>

# Interfaces configuration

By default, *Fast DDS* makes use of all active network interfaces found in the system to communicate (note: applies to
[UDP Transport](udp/udp.md#transport-udp-udp) and [TCP Transport](tcp/tcp.md#transport-tcp-tcp)).
However, it is possible for a user to indicate a specific set of network interfaces to be used by the library,
and/or configure some of them in a specific manner.

<a id="netmask-filtering"></a>

<a id="id1"></a>

## Netmask filtering

The standard behaviour in *Fast-DDS* is to attempt data transmission to any remote locator for which a compatible
transport (based on [kind](transport_api.md#transport-transportapi-kind)) is registered.
This may result in a non-optimum resource utilization, as messages could be sent from an interface to an unreachable
destination given a particular network architecture.
In this situation, a user may decide to enable the netmask filtering feature, which would prevent this behavior by only
sending data from a network interface to remote locators within the same subnetwork.

This configuration option can be set at participant, transport and interface levels, and its possible values are:

<a id="netmask-filter-values"></a>

| Value   | Description                                   |
|---------|-----------------------------------------------|
| `ON`    | Enable netmask filtering.                     |
| `OFF`   | Disable netmask filtering.                    |
| `AUTO`  | Use container’s netmask filter configuration. |

An `AUTO` netmask filter configuration means that its effective value will be given by that of
its “container”, which in the case of an [allowlist](#interfaces-allowlist) entry would be the
[transport descriptor](../xml_configuration/transports.md#transportdescriptors) where it is included, and in the case of a transport descriptor
would be the participant where it is registered.

However not all configurations are valid; this is, for example, a transport’s netmask filter configuration cannot
be `OFF` if it is `ON` for the participant where this transport
is registered.
Likewise, the netmask filter configuration for an allowlist entry cannot be `ON` if it is
`OFF` for the transport descriptor where this allowlist is defined.

#### NOTE
Due to implementation details, it is required to set ignore_non_matching_locators to true
(see [Matching algorithm](announced_locators.md#external-locators-algorithm)) both in [participants](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos)
and [endpoints](../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos) when enabling the netmask filtering feature at participant or transport level
without defining an [allowlist](#interfaces-allowlist) or [blocklist](#interfaces-blocklist).

Additional considerations need to be taken into account when using netmask filtering in combination with
[external locators](announced_locators.md#external-locators).
In particular, it is not possible to enable netmask filtering in all entries of an allowlist when a set of local
external locators (with an *externality* greater than 0) is defined for a [participant](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos)
or [endpoint](../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos).
The reason for this is that a matching remote external locator would then (most likely) be effectively ignored, as no
network interface would be able to reach it according to its network mask.

Netmask filtering can be enabled at participant level both via C++ API or XML configuration:

### C++

```c++
// Configure netmask filtering at participant level
qos.transport().netmask_filter = NetmaskFilterKind::AUTO;
qos.wire_protocol().ignore_non_matching_locators = true; // Required if not defining an allowlist or blocklist
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="CustomTcpParticipantNetmaskFilterParticipant">
        <rtps>
            <ignore_non_matching_locators>true</ignore_non_matching_locators>
            <netmask_filter>ON</netmask_filter>
        </rtps>
    </participant>
</profiles>
```

For socket (UDP/TCP) transport descriptors:

### C++

```c++
// Create a descriptor for the new transport.
auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

// Configure netmask filtering at transport level
udp_transport->netmask_filter = NetmaskFilterKind::AUTO;
qos.wire_protocol().ignore_non_matching_locators = true; // Required if not defining an allowlist or blocklist
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomTcpTransportNetmaskFilter</transport_id>
            <type>TCPv4</type>
            <netmask_filter>ON</netmask_filter>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="CustomTcpTransportNetmaskFilterParticipant">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>CustomTcpTransportNetmaskFilter</transport_id>
            </userTransports>
            <ignore_non_matching_locators>true</ignore_non_matching_locators>
        </rtps>
    </participant>
</profiles>
```

See [Allowlist](#interfaces-allowlist) to learn how to configure netmask filtering for specific network devices.

<a id="interfaces-allowlist"></a>

<a id="allowlist"></a>

## Allowlist

Using *Fast DDS*, it is possible to limit the network interfaces used by [TCP Transport](tcp/tcp.md#transport-tcp-tcp)
and [UDP Transport](udp/udp.md#transport-udp-udp).
This is achieved by adding the interfaces to the [`allowlist`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_allowlistE) field in the
[TCPTransportDescriptor](tcp/tcp.md#transport-tcp-transportdescriptor) or [UDPTransportDescriptor](udp/udp.md#transport-udp-transportdescriptor).
Thus, the communication interfaces used by the [DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) whose [`TransportDescriptorInterface`](../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) defines an
[`allowlist`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_allowlistE) is limited to the interfaces defined in that list, therefore
avoiding the use of the rest of the network interfaces available in the system.
The interfaces in [`allowlist`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_allowlistE) can be specified both by IP address
or interface name.
Additionally, each entry added to the allowlist may specify a [netmask filter](#netmask-filtering)
configuration value (`AUTO` by default).

For example:

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

// Add allowed interface by device name
udp_transport->interface_allowlist.emplace_back("eth0", NetmaskFilterKind::OFF);

// Add allowed interface by IP address (using default netmask filter AUTO)
udp_transport->interface_allowlist.emplace_back("127.0.0.1");

// Add allowed interface with explicit AllowedNetworkInterface construction
AllowedNetworkInterface another_allowed_interface("docker0", NetmaskFilterKind::OFF);
udp_transport->interface_allowlist.emplace_back(another_allowed_interface);

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(udp_transport);

// Avoid using the builtin transports
qos.transport().use_builtin_transports = false;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomTcpTransportAllowlist</transport_id>
            <type>TCPv4</type>
            <interfaces>
                <allowlist>
                    <interface name="wlp59s0" netmask_filter="ON"/>
                    <interface name="192.168.1.10" netmask_filter="OFF"/>
                </allowlist>
            </interfaces>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="CustomTcpTransportAllowlistParticipant">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>CustomTcpTransportAllowlist</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

#### IMPORTANT
If none of the values in the transport descriptor’s allowlist match the interfaces on the host, then all the
interfaces in the allowlist are filtered out and therefore no communication will be established through that
transport.

#### NOTE
**[Pro]** IPv6 addresses are compared without their zone index (e.g. `fe80::1%eth0` matches `fe80::1` on any interface).
Hence, link-local interfaces should be specified by interface name.
In [TCPv6](tcp/tcp.md#transport-tcp-ipv6-addresses), the allowlist also selects the interface used to connect to remote
link-local addresses.

<a id="interfaces-blocklist"></a>

<a id="blocklist"></a>

## Blocklist

Apart from defining a list of [allowed interfaces](#interfaces-allowlist), it is also possible to define a list of
interfaces that should be blocked.
This is accomplished through the [`blocklist`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_blocklistE) field present in the
[TCPTransportDescriptor](tcp/tcp.md#transport-tcp-transportdescriptor) or [UDPTransportDescriptor](udp/udp.md#transport-udp-transportdescriptor).

#### NOTE
This list takes precedence over the allowlist, so if a network interface is in both lists, it will be blocked.

The interfaces in [`blocklist`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_blocklistE) can be specified both by IP address
or interface name.
**[Pro]** As in the [allowlist](#interfaces-allowlist), IPv6 addresses are compared without their zone index.

For example:

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

// Add blocked interface by device name
udp_transport->interface_blocklist.emplace_back("docker0");

// Add blocked interface by IP address
udp_transport->interface_blocklist.emplace_back("127.0.0.1");

// Add blocked interface with explicit BlockedNetworkInterface construction
BlockedNetworkInterface another_blocked_interface("eth0");
udp_transport->interface_blocklist.emplace_back(another_blocked_interface);

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(udp_transport);

// Avoid using the builtin transports
qos.transport().use_builtin_transports = false;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomTcpTransportBlocklist</transport_id>
            <type>TCPv4</type>
            <interfaces>
                <blocklist>
                    <interface name="127.0.0.1"/>
                    <interface name="docker0"/>
                </blocklist>
            </interfaces>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="CustomTcpTransportBlocklistParticipant">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>CustomTcpTransportBlocklist</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```
