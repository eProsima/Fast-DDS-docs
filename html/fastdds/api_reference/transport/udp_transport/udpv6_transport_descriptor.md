<a id="api-udpv6-transport-descriptor"></a>

<a id="udpv6transportdescriptor"></a>

# UDPv6TransportDescriptor

### struct UDPv6TransportDescriptor : public eprosima::fastdds::rtps::[UDPTransportDescriptor](udp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptorE)

UDPv6 Transport configuration The kind value for [UDPv6TransportDescriptor](#structeprosima_1_1fastdds_1_1rtps_1_1UDPv6TransportDescriptor) is given by `eprosima::fastdds::rtps::LOCATOR_KIND_UDPv6`. 

Subclassed by eprosima::fastdds::rtps::TSN_UDPv6TransportDescriptor

### Public Functions

### virtual ~UDPv6TransportDescriptor() = default

Destructor. 

### virtual [TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const override

Factory method pattern. It will create and return a [TransportInterface](../transport_generic_interfaces/transport_interface.md#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) corresponding to this descriptor. This provides an interface to the NetworkFactory to create the transports without the need to know about their type 

### UDPv6TransportDescriptor()

Constructor. 

### UDPv6TransportDescriptor(const [UDPv6TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24UDPv6TransportDescriptor24UDPv6TransportDescriptorERK24UDPv6TransportDescriptor) &t) = default

Copy constructor. 

### [UDPv6TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24UDPv6TransportDescriptorE) &operator=(const [UDPv6TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24UDPv6TransportDescriptorE) &t) = default

Copy assignment.
