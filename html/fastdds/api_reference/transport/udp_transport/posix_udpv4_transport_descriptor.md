<a id="api-posix-udpv4-transport-descriptor"></a>

<a id="posixudpv4transportdescriptor"></a>

# PosixUDPv4TransportDescriptor

### struct PosixUDPv4TransportDescriptor : public eprosima::fastdds::rtps::[SocketTransportDescriptor](../transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptorE)

UDPv4 transport over the socket functions of POSIX only, without asio or operating system extensions, for the platforms that have nothing else, such as those of the FACE General Purpose profile.

Its locators are those of the UDPv4 transport (LOCATOR_KIND_UDPv4), so it communicates with any RTPS implementation over UDPv4; a participant uses either one or the other.

POSIX has no way to list the addresses of the host, so the transport uses:
* the loopback address;
* the addresses the host sends unicast and multicast traffic from, which it asks the operating system for;
* the addresses of the participant’s unicast locators, for the others (secondary interfaces of hosts with several).

The interface allowlist, blocklist and whitelist, netmask filtering and the changes of the network interfaces are not supported. 

### Public Functions

### PosixUDPv4TransportDescriptor()

Constructor. 

### PosixUDPv4TransportDescriptor(const [PosixUDPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps29PosixUDPv4TransportDescriptor29PosixUDPv4TransportDescriptorERK29PosixUDPv4TransportDescriptor) &t) = default

Copy constructor. 

### [PosixUDPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps29PosixUDPv4TransportDescriptorE) &operator=(const [PosixUDPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps29PosixUDPv4TransportDescriptorE) &t) = default

Copy assignment. 

### virtual ~PosixUDPv4TransportDescriptor() = default

Destructor. 

### virtual [TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const override

Creates a POSIX UDPv4 transport with this configuration.

* **Returns:**
  The new transport, which the caller owns. 

### bool operator==(const [PosixUDPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps29PosixUDPv4TransportDescriptorE) &t) const

Comparison operator. 

### Public Members

### bool non_blocking_send = false

Whether the send operations do not block, as in the UDP transport.

When set to true, a send operation returns immediately if the buffer of the socket is full, and no error is returned to the upper layer: the application behaves as if the datagram was sent and lost (i.e. the throughput may be reduced). This is useful on high-frequency best-effort writers.

When set to false, a send operation blocks until the buffer of the socket has space for the datagram, or until the maximum blocking time of the write is over. This may hinder performance on high-frequency writers.
