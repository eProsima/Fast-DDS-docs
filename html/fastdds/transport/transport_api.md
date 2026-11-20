<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transport-transportapi"></a>

<a id="transport-api"></a>

# Transport API

The following diagram presents the classes defined on the transport API of *eProsima Fast DDS*.
It shows the abstract API interfaces, and the classes required to implement a transport.

```plantuml
hide empty members
interface TransportDescriptorInterface
{
    +uint32_t maxMessageSize
    +uint32_t maxInitialPeersRange
}
interface TransportInterface
{
    #int32_t transport_kind_
}
class Locator
{
    +int32_t kind
    +uint32_t port
    +octet[16] address
}
TransportDescriptorInterface <|-- CustomTransportDescriptor
TransportInterface <|-- CustomTransport
CustomTransport <--right CustomTransportDescriptor : create
Locator <--right TransportInterface
```

*Transport API diagram*

<a id="transport-transportapi-transportdescriptor"></a>

<a id="transportdescriptorinterface"></a>

## TransportDescriptorInterface

Any class that implements the [`TransportDescriptorInterface`](../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) is known as a `TransportDescriptor`.
It acts as a *builder* for a given transport, meaning that is allows to configure the transport,
and then a new [Transport](#transport-transportapi-transport) can be built according to this configuration
using its [`create_transport`](../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4NK8eprosima7fastdds4rtps28TransportDescriptorInterface16create_transportEv) factory member function.

<a id="data-members"></a>

### Data members

The TransportDescriptorInterface defines the following data members:

| Member                                                                                                                                                                                                 | Data type   | Description                                              |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------|----------------------------------------------------------|
| [`maxMessageSize`](../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterface14maxMessageSizeE)             | `uint32_t`  | Maximum size of a single message in the transport.       |
| [`maxInitialPeersRange`](../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterface20maxInitialPeersRangeE) | `uint32_t`  | Number of channels opened with each initial remote peer. |

Any implementation of [TransportDescriptorInterface](#transport-transportapi-transportdescriptor) should add as many
data members as required to full configure the transport it describes.

<a id="transport-transportapi-transport"></a>

<a id="transportinterface"></a>

## TransportInterface

A `Transport` is any class that implements the [`TransportInterface`](../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE).
It is the object that actually performs the message distribution over a physical transport.

Each `Transport` class defines its own [`kind`](../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4NK8eprosima7fastdds4rtps18TransportInterface4kindEv), a unique identifier that is used to
check the compatibility of a [Locator](#transport-transportapi-locator) with a Transport, i.e.,
determine whether a Locator refers to a Transport or not.

Applications do not create the `Transport` instance themselves.
Instead, applications use a `TransportDescriptor` instance to configure the desired transport, and add
this configured instance to the list of user-defined transports of the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
The DomainParticipant will use the factory function on the `TransportDescriptor`
to create the `Transport` when required.

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

<a id="id1"></a>

### Data members

The TransportInterface defines the following data members:

<a id="transport-transportapi-kind"></a>

| Member            | Data type   | Description                              |
|-------------------|-------------|------------------------------------------|
| `transport_kind_` | `int32_t`   | Unique identifier of the transport type. |

#### NOTE
`transport_kind_` is a protected data member for internal use.
It cannot be accessed nor modified from the public API.
However, users that are implementing a custom Transport need to fill it with a unique constant value
in the new implementation.

Currently the following identifiers are used in *Fast DDS*:

| Identifier                                                                                         |   Value | Transport type                                                                                 |
|----------------------------------------------------------------------------------------------------|---------|------------------------------------------------------------------------------------------------|
| [`LOCATOR_KIND_RESERVED`](../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_RESERVED) |       0 | None. Reserved value for internal use.                                                         |
| [`LOCATOR_KIND_UDPv4`](../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_UDPv4)       |       1 | [UDP Transport](udp/udp.md#transport-udp-udp) over IPv4.                                       |
| [`LOCATOR_KIND_UDPv6`](../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_UDPv6)       |       2 | [UDP Transport](udp/udp.md#transport-udp-udp) over IPv6.                                       |
| [`LOCATOR_KIND_TCPv4`](../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_TCPv4)       |       4 | [TCP Transport](tcp/tcp.md#transport-tcp-tcp) over IPv4.                                       |
| [`LOCATOR_KIND_TCPv6`](../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_TCPv6)       |       8 | [TCP Transport](tcp/tcp.md#transport-tcp-tcp) over IPv6.                                       |
| [`LOCATOR_KIND_SHM`](../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_SHM)           |      16 | [Shared Memory Transport](shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory). |

<a id="transport-transportapi-locator"></a>

<a id="locator"></a>

## Locator

A [`Locator_t`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) uniquely identifies a communication channel with a remote peer for a particular transport.
For example, on UDP transports, the Locator will contain the information of the IP address and port
of the remote peer.

The Locator class is not abstract, and no specializations are implemented for each transport type.
Instead, transports should map the data members of the Locator class to their own channel identification
concepts.
For example, on [Shared Memory Transport](shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory) the [`address`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t7addressE) contains a unique ID
for the local host, and the [`port`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t4portE) represents the shared ring buffer used to communicate buffer
descriptors.

Please refer to [Listening Locators](listening_locators.md#listening-locators) for more information about how to configure
DomainParticipant to listen to incoming traffic.

<a id="id2"></a>

### Data members

The Locator defines the following data members:

| Member                                                                                                          | Data type   | Description                              |
|-----------------------------------------------------------------------------------------------------------------|-------------|------------------------------------------|
| [`kind`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t4kindE)       | `int32_t`   | Unique identifier of the transport type. |
| [`port`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t4portE)       | `uint32_t`  | The channel *port*.                      |
| [`address`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t7addressE) | `octet[16]` | The channel *address*.                   |

In TCP, the port of the locator is divided into a physical and a logical port.

* The *physical port* is the port used by the network device, the real port that the operating system understands.
  It is stored in the two least significant bytes of the member [`port`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t4portE).
* The *logical port* is the RTPS port.
  It is used by the RTPS protocol to distinguish different entities.
  It is stored in the two most significant bytes of the member [`port`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t4portE).

In TCP, this distinction allows for several DDS applications using different RTPS ports (*logical ports*) to share the
same *physical port*, thus only requiring for a single port to be opened for all communications.
In UDP there is only the *physical port*, which is also the RTPS port, and is stored in the two least significant bytes
of the member [`port`](../api_reference/rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_t4portE).

The locator address, represented in 16 bytes, is managed differently depending on whether the protocol used is IPv4 or
IPv6.

* The IPv6 address uses the 16 available bytes to represent the address, which can be a global unicast, a unique
  local, or a link-local address.
  **[Pro]** The locator does not carry the zone index of link-local addresses, as it only has meaning on the local host.
  The transport selects it when needed (see [TCPv6 Addresses and Interfaces](tcp/tcp.md#transport-tcp-ipv6-addresses)).
* The IPv4 address splits those 16 bytes in the following three sections, ordered from least to greatest significance:
  - 4 bytes LAN IP: Local subnet identification (UDP and TCP).
  - 4 bytes WAN IP: Public IP (TCP only).
  - 8 bytes unused.

```idl
                        Locator IPv4 address
+--------+-----------------------------+-----------------------------+
| Unused | WAN address (62.128.41.210) | LAN address (192.168.0.113) |
+--------+-----------------------------+-----------------------------+
 8 bytes       (TCP only) 4 bytes                 4 bytes


                        Locator IPv6 address
+--------------------------------------------------------------------+
|          Address (2001:0000:130F:0000:0000:09C0:876A:130B)         |
+--------------------------------------------------------------------+
                              16 bytes
```

Check how to manipulate the WAN address in the [TCP IPv4 transport descriptor api](../api_reference/transport/tcp_transport/tcpv4_transport_descriptor.md#api-tcpv4-transport-descriptor)
section.

<a id="transport-transportapi-iplocator"></a>

<a id="configuring-ip-locators-with-iplocator"></a>

### Configuring IP locators with IPLocator

`IPLocator` is an auxiliary static class that offers methods to manipulate IP based locators.
It is convenient when setting up a new [UDP Transport](udp/udp.md#transport-udp-udp) or [TCP Transport](tcp/tcp.md#transport-tcp-tcp),
as it simplifies setting IPv4 and IPv6 addresses, or manipulating ports.

For example, normally users configure the physical port and do not need to worry about logical ports.
However, `IPLocator` allows to manage them if needed.

```c++
// We will configure a TCP locator with IPLocator
Locator_t locator;

// Get & set the physical port
uint16_t physical_port = IPLocator::getPhysicalPort(locator);
IPLocator::setPhysicalPort(locator, 5555);

// On TCP locators, we can get & set the logical port
uint16_t logical_port = IPLocator::getLogicalPort(locator);
IPLocator::setLogicalPort(locator, 7400);

// Set WAN address
IPLocator::setWan(locator, "80.88.75.55");
```

*Fast DDS* also allows to specify locator addresses using names.
When an address is specified by a name, *Fast DDS* will query the known hosts and available DNS servers to try to
resolve the IP address.
This address will in turn be used to create the listening locator in the case of *server*, or as the address of the
remote *server* in the case of *clients* (and *servers* that connect to other *servers*).

### C++

```c++
Locator_t locator;
auto response = eprosima::fastdds::rtps::IPLocator::resolveNameDNS("localhost");
// Get the first returned IPv4
if (response.first.size() > 0)
{
    IPLocator::setIPv4(locator, response.first.begin()->data());
    locator.port = 11811;
}
// Use the locator to create server or client
```

### XML

```xml
<locator>
    <udpv4>
        <port>11811</port>
        <address>localhost</address>
    </udpv4>
</locator>
```

#### NOTE
**[Pro]** For TCPv6 addresses, the zone index suffix (e.g. `fe80::1%eth0`) is accepted but ignored.
For more information, see [TCPv6 Addresses and Interfaces](tcp/tcp.md#transport-tcp-ipv6-addresses).

<a id="transport-transportapi-chaining"></a>

<a id="chaining-of-transports"></a>

## Chaining of transports

There are use cases where the user needs to pre-process out-coming information before being sent to network and also
the incoming information after being received.
*Transport API* offers two interfaces for implementing this kind of functionality: [`ChainingTransportDescriptor`](../api_reference/transport/chaining_transport/chaining_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps27ChainingTransportDescriptorE)
and [`ChainingTransport`](../api_reference/transport/chaining_transport/chaining_transport.md#_CPPv4N8eprosima7fastdds4rtps17ChainingTransportE).

```plantuml
hide empty members
interface TransportDescriptorInterface
{
    +uint32_t maxMessageSize
    +uint32_t maxInitialPeersRange
}
interface TransportInterface
{
    #int32_t transport_kind_
}
interface ChainingTransportDescriptor
{
    +std::shared_ptr<TransportDescriptorInterface> low_level_descriptor
}
interface ChainingTransport
{
    #std::unique_ptr<TransportInterface> low_level_transport_
    {abstract} bool send(...)
    {abstract} void receive(...)
}
TransportDescriptorInterface <|-- ChainingTransportDescriptor
TransportInterface <|-- ChainingTransport
ChainingTransportDescriptor <|-- CustomChainingTransportDescriptor
ChainingTransport <|-- CustomChainingTransport
CustomChainingTransport <-- CustomChainingTransportDescriptor : create
CustomChainingTransportDescriptor "1" *-- "1" TransportDescriptorInterface : contains
CustomChainingTransport "1" *-- "1" TransportInterface : contains
CustomChainingTransportDescriptor --- UDPv4TransportDescriptor : example
CustomChainingTransport ---  UDPv4Transport : example
```

These extensions allow to implement a new Transport which depends on another one (called here as
`low_level_transport_`).
The user can override the [`send()`](../api_reference/transport/chaining_transport/chaining_transport.md#_CPPv4N8eprosima7fastdds4rtps17ChainingTransport4sendEPN7fastdds4rtps14SenderResourceERKNSt6vectorI13NetworkBufferEENSt8uint32_tEPN7fastdds4rtps16LocatorsIteratorEPN7fastdds4rtps16LocatorsIteratorERKNSt6chrono12steady_clock10time_pointE) function, pre-processing the out-coming buffer before calling
the associated `low_level_transport_`.
Also, when a incoming buffer arrives to the `low_level_transport_`, this one calls the overridden
[`receive()`](../api_reference/transport/chaining_transport/chaining_transport.md#_CPPv4N8eprosima7fastdds4rtps17ChainingTransport7receiveEP26TransportReceiverInterfacePKN7fastdds4rtps5octetENSt8uint32_tERKN7fastdds4rtps9Locator_tERKN7fastdds4rtps9Locator_tE) function to allow to pre-process the buffer.

<a id="transport-transportapi-chaining-descriptor"></a>

<a id="chainingtransportdescriptor"></a>

### ChainingTransportDescriptor

Implementing [`ChainingTransportDescriptor`](../api_reference/transport/chaining_transport/chaining_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps27ChainingTransportDescriptorE) allows to configure the new Transport and set the
`low_level_transport_` on which it depends.
The associated `low_level_transport_` can be any transport which inherits from
[`TransportInterface`](../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) (including another [`ChainingTransport`](../api_reference/transport/chaining_transport/chaining_transport.md#_CPPv4N8eprosima7fastdds4rtps17ChainingTransportE)).

The [`ChainingTransportDescriptor`](../api_reference/transport/chaining_transport/chaining_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps27ChainingTransportDescriptorE) defines the following data members:

| Member                                                                                                                                                                                     | Data type                                       | Description                                         |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------|-----------------------------------------------------|
| [`low_level_descriptor`](../api_reference/transport/chaining_transport/chaining_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps27ChainingTransportDescriptor20low_level_descriptorE) | `std::shared_ptr<TransportDescriptorInterface>` | Transport descriptor of the `low_level_transport_`. |

User has to specify the `low_level_transport_` in the definition of its new custom transport.

```c++
DomainParticipantQos qos;

auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

// Create a descriptor for the new transport.
// The low level transport will be a UDPv4Transport.
auto custom_transport = std::make_shared<CustomChainingTransportDescriptor>(udp_transport);

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(custom_transport);

// Avoid using the default transport
qos.transport().use_builtin_transports = false;
```

<a id="chainingtransport"></a>

### ChainingTransport

This interface forces the user to implement [`send()`](../api_reference/transport/chaining_transport/chaining_transport.md#_CPPv4N8eprosima7fastdds4rtps17ChainingTransport4sendEPN7fastdds4rtps14SenderResourceERKNSt6vectorI13NetworkBufferEENSt8uint32_tEPN7fastdds4rtps16LocatorsIteratorEPN7fastdds4rtps16LocatorsIteratorERKNSt6chrono12steady_clock10time_pointE) and [`receive()`](../api_reference/transport/chaining_transport/chaining_transport.md#_CPPv4N8eprosima7fastdds4rtps17ChainingTransport7receiveEP26TransportReceiverInterfacePKN7fastdds4rtps5octetENSt8uint32_tERKN7fastdds4rtps9Locator_tERKN7fastdds4rtps9Locator_tE)
functions.
The idea is to pre-process the buffer and after, call to the next level.

```c++
class CustomChainingTransport : public eprosima::fastdds::rtps::ChainingTransport
{

public:

    CustomChainingTransport(
            const CustomChainingTransportDescriptor& descriptor)
        : ChainingTransport(descriptor)
        , descriptor_(descriptor)
    {
    }

    eprosima::fastdds::rtps::TransportDescriptorInterface* get_configuration()
    {
        return &descriptor_;
    }

    bool send(
            eprosima::fastdds::rtps::SenderResource* low_sender_resource,
            const std::vector<eprosima::fastdds::rtps::NetworkBuffer>& buffers,
            uint32_t total_bytes,
            eprosima::fastdds::rtps::LocatorsIterator* destination_locators_begin,
            eprosima::fastdds::rtps::LocatorsIterator* destination_locators_end,
            const std::chrono::steady_clock::time_point& timeout) override
    {
        //
        // Preprocess outcoming buffer.
        //

        // Call low level transport
        return low_sender_resource->send(buffers, total_bytes, destination_locators_begin,
                       destination_locators_end, timeout, 0);
    }

    bool send_w_priority(
            eprosima::fastdds::rtps::SenderResource* low_sender_resource,
            const std::vector<eprosima::fastdds::rtps::NetworkBuffer>& buffers,
            uint32_t total_bytes,
            eprosima::fastdds::rtps::LocatorsIterator* destination_locators_begin,
            eprosima::fastdds::rtps::LocatorsIterator* destination_locators_end,
            const std::chrono::steady_clock::time_point& timeout,
            int32_t transport_priority) override
    {
        //
        // Preprocess outcoming buffer.
        //

        // Call low level transport
        return low_sender_resource->send(buffers, total_bytes, destination_locators_begin,
                       destination_locators_end, timeout, transport_priority);
    }

    void receive(
            eprosima::fastdds::rtps::TransportReceiverInterface* next_receiver,
            const eprosima::fastdds::rtps::octet* receive_buffer,
            uint32_t receive_buffer_size,
            const eprosima::fastdds::rtps::Locator_t& local_locator,
            const eprosima::fastdds::rtps::Locator_t& remote_locator) override
    {
        //
        // Preprocess incoming buffer.
        //

        // Call upper level
        next_receiver->OnDataReceived(receive_buffer, receive_buffer_size, local_locator, remote_locator);
    }

private:

    CustomChainingTransportDescriptor descriptor_;
};
```
