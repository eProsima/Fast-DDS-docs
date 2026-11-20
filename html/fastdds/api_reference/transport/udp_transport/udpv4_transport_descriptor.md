<a id="api-udpv4-transport-descriptor"></a>

<a id="udpv4transportdescriptor"></a>

# UDPv4TransportDescriptor

### struct UDPv4TransportDescriptor : public eprosima::fastdds::rtps::[UDPTransportDescriptor](udp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22UDPTransportDescriptorE)

UDPv4 Transport configuration The kind value for [UDPv4TransportDescriptor](#structeprosima_1_1fastdds_1_1rtps_1_1UDPv4TransportDescriptor) is given by `eprosima::fastdds::rtps::LOCATOR_KIND_UDPv4`. 

Subclassed by eprosima::fastdds::rtps::TSN_UDPv4TransportDescriptor

### Public Functions

### virtual ~UDPv4TransportDescriptor() = default

Destructor. 

### virtual [TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const override

Factory method pattern. It will create and return a [TransportInterface](../transport_generic_interfaces/transport_interface.md#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) corresponding to this descriptor. This provides an interface to the NetworkFactory to create the transports without the need to know about their type 

### UDPv4TransportDescriptor()

Constructor. 

### UDPv4TransportDescriptor(const [UDPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24UDPv4TransportDescriptor24UDPv4TransportDescriptorERK24UDPv4TransportDescriptor) &t) = default

Copy constructor. 

### [UDPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24UDPv4TransportDescriptorE) &operator=(const [UDPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24UDPv4TransportDescriptorE) &t) = default

Copy assignment.
