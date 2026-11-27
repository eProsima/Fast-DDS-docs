<a id="api-socket-transport-descriptor"></a>

<a id="sockettransportdescriptor"></a>

# SocketTransportDescriptor

### struct SocketTransportDescriptor : public eprosima::fastdds::rtps::[PortBasedTransportDescriptor](port_based_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps28PortBasedTransportDescriptorE)

Virtual base class for the data type used to define configuration of transports using sockets.

* `sendBufferSize:` size of the sending buffer of the socket (in octets).
* `receiveBufferSize:` size of the receiving buffer of the socket (in octets).
* `interfaceWhiteList:` list of allowed interfaces.
* `TTL:` time to live, in number of hops. 

Subclassed by [eprosima::fastdds::rtps::PosixUDPv4TransportDescriptor](../udp_transport/posix_udpv4_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1PosixUDPv4TransportDescriptor), [eprosima::fastdds::rtps::TCPTransportDescriptor](../tcp_transport/tcp_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1TCPTransportDescriptor), [eprosima::fastdds::rtps::UDPTransportDescriptor](../udp_transport/udp_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1UDPTransportDescriptor), eprosima::fastdds::rtps::test_UDPv4TransportDescriptor

### Public Functions

### inline SocketTransportDescriptor(std::uint32_t maximumMessageSize, std::uint32_t maximumInitialPeersRange)

Constructor. 

### SocketTransportDescriptor(const [SocketTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor25SocketTransportDescriptorERK25SocketTransportDescriptor) &t) = default

Copy constructor. 

### [SocketTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptorE) &operator=(const [SocketTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptorE) &t) = default

Copy assignment. 

### virtual ~SocketTransportDescriptor() = default

Destructor. 

### inline virtual std::uint32_t min_send_buffer_size() const override

Returns the minimum size required for a send operation. 

### inline bool operator==(const [SocketTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptorE) &t) const

Comparison operator. 

### Public Members

### std::uint32_t sendBufferSize

Length of the send buffer. 

### std::uint32_t receiveBufferSize

Length of the receive buffer. 

### std::vector<std::string> interfaceWhiteList

Allowed interfaces in an IP or device name string format. 

### NetmaskFilterKind netmask_filter

Transport’s netmask filter configuration. 

### std::vector<AllowedNetworkInterface> interface_allowlist

Allowed interfaces in an IP or device name string format, each with a specific netmask filter configuration. 

### std::vector<BlockedNetworkInterface> interface_blocklist

Blocked interfaces in an IP or device name string format. 

### std::uint8_t TTL

Specified time to live (8bit - 255 max TTL) 

### constexpr std::uint8_t eprosima::fastdds::rtps::s_defaultTTL = 1

Default time to live (TTL)
