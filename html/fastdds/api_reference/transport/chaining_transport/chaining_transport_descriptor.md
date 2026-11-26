<a id="api-chaining-transport-descriptor"></a>

<a id="chainingtransportdescriptor"></a>

# ChainingTransportDescriptor

### struct ChainingTransportDescriptor : public eprosima::fastdds::rtps::[TransportDescriptorInterface](../transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE)

Base class for the descriptors of chaining transports. A chaining transport allows for the manipulation of data before sending or after receiving from another transport.

Transport configuration:

* low_level_descriptor: Descriptor for lower level transport. 

Subclassed by eprosima::fastdds::rtps::HeaderReductionTransportDescriptor, eprosima::fastdds::rtps::SourceTimestampTransportDescriptor

### Public Functions

### inline virtual std::uint32_t min_send_buffer_size() const override

Returns the minimum size required for a send operation. 

### inline virtual std::uint32_t max_message_size() const override

Returns the maximum size expected for received messages. 

### Public Members

### std::shared_ptr<[TransportDescriptorInterface](../transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE)> low_level_descriptor

Descriptor for lower level transport.
