<a id="api-transport-descriptor-interface"></a>

<a id="transportdescriptorinterface"></a>

# TransportDescriptorInterface

### struct TransportDescriptorInterface

Virtual base class for the data type used to define transport configuration. It acts as a builder for a given transport meaning that it allows to configure the transport, and then a new Transport can be built according to this configuration using its [create_transport()](#structeprosima_1_1fastdds_1_1rtps_1_1TransportDescriptorInterface_1a202ffb0a1032bb1838bf2dfa95e89630) factory member function.

* maxMessageSize: maximum size of a single message in the transport.
* maxInitialPeersRange: number of channels opened with each initial remote peer. 

Subclassed by [eprosima::fastdds::rtps::ChainingTransportDescriptor](../chaining_transport/chaining_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1ChainingTransportDescriptor), [eprosima::fastdds::rtps::PortBasedTransportDescriptor](port_based_transport_descriptor.md#classeprosima_1_1fastdds_1_1rtps_1_1PortBasedTransportDescriptor)

### Public Functions

### inline TransportDescriptorInterface(std::uint32_t maximumMessageSize, std::uint32_t maximumInitialPeersRange)

Constructor. 

### inline TransportDescriptorInterface(const [TransportDescriptorInterface](#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterface28TransportDescriptorInterfaceERK28TransportDescriptorInterface) &t)

Copy constructor. 

### inline [TransportDescriptorInterface](#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) &operator=(const [TransportDescriptorInterface](#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) &t)

Copy assignment. 

### virtual ~TransportDescriptorInterface() = default

Destructor. 

### virtual [TransportInterface](transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const = 0

Factory method pattern. It will create and return a [TransportInterface](transport_interface.md#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) corresponding to this descriptor. This provides an interface to the NetworkFactory to create the transports without the need to know about their type 

### virtual std::uint32_t min_send_buffer_size() const = 0

Returns the minimum size required for a send operation. 

### inline virtual std::uint32_t max_message_size() const

Returns the maximum size expected for received messages. 

### inline virtual std::uint32_t max_initial_peers_range() const

Returns the maximum number of opened channels for each initial remote peer (maximum number of guessed initial peers to try to connect) 

### inline bool operator==(const [TransportDescriptorInterface](#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) &t) const

Comparison operator. 

### inline void lock()

Lock internal mutex (for Fast-DDS internal use) 

### inline void unlock()

Unlock internal mutex (for Fast-DDS internal use) 

### Public Members

### std::uint32_t maxMessageSize

Maximum size of a single message in the transport. 

### std::uint32_t maxInitialPeersRange

Number of channels opened with each initial remote peer.
