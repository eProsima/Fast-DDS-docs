<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="ip-mobility"></a>

<a id="ip-mobility-pro"></a>

# IP mobility **[Pro]**

DDS [Simple Discovery](../../discovery/simple.md#simple-disc-settings) relies on well-known multicast addresses and ports to relay
the Participant announcement messages (see [Discovery phases](../../discovery/discovery.md#disc-phases)).
Such Participant announcement includes information about the unicast address-port pairs (a.k.a. locators) where
the Participant is expecting to receive incoming metatraffic data.
The list with these unicast locators is automatically initialized taking into account the network interfaces that
are available when the *Fast DDS* DomainParticipant is enabled.

Fast DDS Pro detects changes in the system’s IP addresses, and automatically updates them through the discovery
protocol, so other participants start sending data to the new addresses.

#### IMPORTANT
This feature is only officially supported for UDPv4 Transport.

For TCP transports, IP mobility is achieved through the [TCP Keep Alive](#tcp-keep-alive) mechanism.
In this case, interface filtering (see [Interface filtering](#ip-mobility-interface-filtering)) is not supported yet.

This feature is only available in Windows and Linux.
MacOS will be supported in future releases.

<a id="prerequisites"></a>

## Prerequisites

This feature is intended to be used when *Fast DDS* automatically sets the listening unicast locators.
Consequently, all the locator lists in [`wire_protocol()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos13wire_protocolEv) must be empty when the participant
is created.
If any of them is not empty, no updates will be transmitted to other participants, and communication with them will
stop.

These locator lists are:

* [`metatrafficUnicastLocatorList`](../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes29metatrafficUnicastLocatorListE)
* [`metatrafficMulticastLocatorList`](../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes31metatrafficMulticastLocatorListE)
* [`default_unicast_locator_list`](../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos28default_unicast_locator_listE)
* [`default_multicast_locator_list`](../../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos30default_multicast_locator_listE)

Please refer to [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) for more information about these attributes.

#### NOTE
Be aware of the remote locators’ collections limits set within the [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos)
(please refer to [RemoteLocatorsAllocationAttributes](../../dds_layer/core/policy/eprosimaExtensions.md#remotelocatorsallocationattributes)).
It is recommended to use the highest number of local addresses found on all the systems belonging to the same
domain.

<a id="ip-mobility-interface-filtering"></a>

<a id="interface-filtering"></a>

## Interface filtering

IP mobility is compatible with the interface filters of the UDP transport descriptor:

* [Allowlist](../../transport/interfaces.md#interfaces-allowlist)
* [Blocklist](../../transport/interfaces.md#interfaces-blocklist)
* [Whitelist](../../transport/whitelist.md#whitelist-interfaces)

Upon a change in the network interfaces, these filters are re-evaluated against the interfaces currently available
in the system, matching entries either by interface name or by IP address.
Thus, an interface that matches the filters and appears (or gets a new IP address) after the participant is enabled
is automatically used for communication.

#### WARNING
Avoid combining interface filters with user defined unicast listening locators, both for metatraffic and user
traffic.
Upon participant creation, a user defined unicast locator whose interface is filtered out by the transport
(e.g. because the interface is not present yet) is discarded, as no receiver can be created for it.
Since only internally generated locators are recomputed upon a network interfaces change, such locator is never
restored, and the participant never listens on it, even if the interface becomes available later.

<a id="tcp-keep-alive"></a>

<a id="tcp-keep-alive-pro"></a>

## TCP Keep Alive **[Pro]**

Fast DDS Pro additionally provides a TCP keep-alive mechanism to detect stalled TCP connections and recover
from link failures more reliably.
When enabled, the TCP transport periodically sends keep-alive requests on established TCP channels, and monitors
activity to determine whether a connection is still responsive.
Keep alive requests are sent only when no incoming data has been received for a specified interval.

If the configured timeout is reached without observing activity after a keep-alive request, the channel is
considered broken and is closed, allowing Fast DDS to reconnect and restore communication.

Note that this feature operates at the application level, independently of any TCP keep-alive settings at the OS level.
This allows for more fine-grained control over connection monitoring and recovery within Fast DDS.
It also ensures consistent behavior across different operating systems and increases portability.

Keep alive is disabled by default and can be configured through the following parameters in
[`TCPTransportDescriptor`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE):

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
        <transport_descriptor>
            <transport_id>keep_alive_transport</transport_id>
            <type>TCPv4</type>
            <keep_alive_frequency_ms>5000</keep_alive_frequency_ms>
            <keep_alive_timeout_ms>25000</keep_alive_timeout_ms>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
```

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
auto tcp_transport = std::make_shared<TCPv4TransportDescriptor>();

// Configure keep alive options
tcp_transport->keep_alive_frequency_ms = 5000;  // Send a keep-alive message every 5 seconds when connection is idle
tcp_transport->keep_alive_timeout_ms = 2000;  // Consider connection lost if no response is received within 2 seconds
tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};  // Configure thread settings

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(tcp_transport);
```

#### NOTE
This feature is intended for TCP transports and only applies to connected TCP channels.
