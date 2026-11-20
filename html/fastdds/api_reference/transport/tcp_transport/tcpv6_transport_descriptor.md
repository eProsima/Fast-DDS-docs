<a id="api-tcpv6-transport-descriptor"></a>

<a id="tcpv6transportdescriptor"></a>

# TCPv6TransportDescriptor

### struct TCPv6TransportDescriptor : public eprosima::fastdds::rtps::[TCPTransportDescriptor](tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE)

TCPv6 Transport configuration The kind value for [TCPv6TransportDescriptor](#structeprosima_1_1fastdds_1_1rtps_1_1TCPv6TransportDescriptor) is given by `eprosima::fastdds::rtps::LOCATOR_KIND_TCPv6`. 

### Public Functions

### virtual ~TCPv6TransportDescriptor() = default

Destructor. 

### virtual [TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const override

Factory method pattern. It will create and return a [TransportInterface](../transport_generic_interfaces/transport_interface.md#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) corresponding to this descriptor. This provides an interface to the NetworkFactory to create the transports without the need to know about their type 

### TCPv6TransportDescriptor()

Constructor. 

### TCPv6TransportDescriptor(const [TCPv6TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv6TransportDescriptor24TCPv6TransportDescriptorERK24TCPv6TransportDescriptor) &t)

Copy constructor. 

### [TCPv6TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv6TransportDescriptorE) &operator=(const [TCPv6TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv6TransportDescriptorE) &t) = default

Copy assignment. 

### bool operator==(const [TCPv6TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv6TransportDescriptorE) &t) const

Comparison operator.
