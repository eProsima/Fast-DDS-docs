<a id="api-udp-transport-descriptor"></a>

<a id="udptransportdescriptor"></a>

# UDPTransportDescriptor

### struct UDPTransportDescriptor : public eprosima::fastdds::rtps::[SocketTransportDescriptor](../transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptorE)

UDP Transport configuration

* `m_output_udp_socket:` source port to use for outgoing datagrams.
* `non_blocking_send:` do not block on send operations. When it is set to true, send operations will return immediately if the buffer is full, but no error will be returned to the upper layer. This means that the application will behave as if the datagram is sent and lost. 

Subclassed by [eprosima::fastdds::rtps::UDPv4TransportDescriptor](udpv4_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1UDPv4TransportDescriptor), [eprosima::fastdds::rtps::UDPv6TransportDescriptor](udpv6_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1UDPv6TransportDescriptor)

### Public Functions

### virtual ~UDPTransportDescriptor() = default

Destructor. 

### UDPTransportDescriptor()

Constructor. 

### UDPTransportDescriptor(const [UDPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptor22UDPTransportDescriptorERK22UDPTransportDescriptor) &t) = default

Copy constructor. 

### [UDPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptorE) &operator=(const [UDPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptorE) &t) = default

Copy assignment. 

### bool operator==(const [UDPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptorE) &t) const

Comparison operator. 

### Public Members

### std::uint16_t m_output_udp_socket

Source port to use for outgoing datagrams. 

### bool non_blocking_send = false

Whether to use non-blocking calls to send_to().

When set to true, calls to send_to() will return immediately if the buffer is full, but no error will be returned to the upper layer. This means that the application will behave as if the datagram is sent but lost (i.e. throughput may be reduced). This value is specially useful on high-frequency best-effort writers.

When set to false, calls to send_to() will block until the network buffer has space for the datagram. This may hinder performance on high-frequency writers.
