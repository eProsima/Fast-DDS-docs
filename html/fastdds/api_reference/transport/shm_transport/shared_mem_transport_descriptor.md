<a id="api-shared-mem-transport-descriptor"></a>

<a id="sharedmemtransportdescriptor"></a>

# SharedMemTransportDescriptor

### struct SharedMemTransportDescriptor : public eprosima::fastdds::rtps::[PortBasedTransportDescriptor](../transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps28PortBasedTransportDescriptorE)

Shared memory transport configuration. The kind is given by eprosima::fastdds::rtps::LOCATOR_KIND_SHM.

* segment_size_: size of the shared memory segment (in octets).
* port_queue_capacity_: size of the listening port (in messages).
* healthy_check_timeout_ms_: timeout for the health check of ports (ms).
* rtps_dump_file_: full path of the protocol dump file. 

Subclassed by [eprosima::fastdds::rtps::test_SharedMemTransportDescriptor](test_shared_mem_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1test__SharedMemTransportDescriptor)

### Public Functions

### virtual ~SharedMemTransportDescriptor() = default

Destructor. 

### virtual [TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const override

Factory method pattern. It will create and return a [TransportInterface](../transport_generic_interfaces/transport_interface.md#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) corresponding to this descriptor. This provides an interface to the NetworkFactory to create the transports without the need to know about their type 

### inline virtual std::uint32_t min_send_buffer_size() const override

Minimum size of the send buffer. 

### SharedMemTransportDescriptor()

Constructor. 

### SharedMemTransportDescriptor(const [SharedMemTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28SharedMemTransportDescriptor28SharedMemTransportDescriptorERK28SharedMemTransportDescriptor) &t) = default

Copy constructor. 

### [SharedMemTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28SharedMemTransportDescriptorE) &operator=(const [SharedMemTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28SharedMemTransportDescriptorE) &t) = default

Copy assignment. 

### inline std::uint32_t segment_size() const

Return the size of the shared memory segment. 

### inline void segment_size(std::uint32_t segment_size)

Set the size of the shared memory segment. 

### inline virtual std::uint32_t max_message_size() const override

Return the maximum size of a single message in the transport (in octets) 

### inline void max_message_size(std::uint32_t max_message_size)

Set the maximum size of a single message in the transport (in octets) 

### inline std::uint32_t port_queue_capacity() const

Return the size of the listening port (in messages) 

### inline void port_queue_capacity(std::uint32_t port_queue_capacity)

Set the size of the listening port (in messages) 

### inline std::uint32_t healthy_check_timeout_ms() const

Return the timeout for the health check of ports (ms) 

### inline void healthy_check_timeout_ms(std::uint32_t healthy_check_timeout_ms)

Set the timeout for the health check of ports (ms) 

### inline std::string rtps_dump_file() const

Return the full path of the protocol dump file. 

### inline void rtps_dump_file(const std::string &rtps_dump_file)

Set the full path of the protocol dump file. 

### inline [ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) dump_thread() const

Return the thread settings for the transport dump thread. 

### inline void dump_thread(const [ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &dump_thread)

Set the thread settings for the transport dump thread. 

### bool operator==(const [SharedMemTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28SharedMemTransportDescriptorE) &t) const

Comparison operator.
