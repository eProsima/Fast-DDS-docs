<a id="api-tcpv4-transport-descriptor"></a>

<a id="tcpv4transportdescriptor"></a>

# TCPv4TransportDescriptor

### struct TCPv4TransportDescriptor : public eprosima::fastdds::rtps::[TCPTransportDescriptor](tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE)

TCPv4 Transport configuration. The kind value for [TCPv4TransportDescriptor](#structeprosima_1_1fastdds_1_1rtps_1_1TCPv4TransportDescriptor) is given by `eprosima::fastdds::rtps::LOCATOR_KIND_TCPv4`.

* `wan_addr:` Public IP address. Peers on a different LAN will use this IP for communications with this host. 

### Public Functions

### virtual ~TCPv4TransportDescriptor() = default

Destructor. 

### virtual [TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const override

Factory method pattern. It will create and return a [TransportInterface](../transport_generic_interfaces/transport_interface.md#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) corresponding to this descriptor. This provides an interface to the NetworkFactory to create the transports without the need to know about their type 

### inline void set_WAN_address(fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) o1, fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) o2, fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) o3, fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) o4)

Set the public IP address. 

### inline void set_WAN_address(const std::string &in_address)

Set the public IP address. 

### inline std::string get_WAN_address()

Get the public IP address. 

### TCPv4TransportDescriptor()

Constructor. 

### TCPv4TransportDescriptor(const [TCPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptor24TCPv4TransportDescriptorERK24TCPv4TransportDescriptor) &t)

Copy constructor. 

### [TCPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptorE) &operator=(const [TCPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptorE) &t)

Copy assignment. 

### bool operator==(const [TCPv4TransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptorE) &t) const

Comparison operator. 

### Public Members

### fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) wan_addr[4]

Public IP address.
