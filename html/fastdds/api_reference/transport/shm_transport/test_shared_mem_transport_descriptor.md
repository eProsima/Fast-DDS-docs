<a id="api-test-shared-mem-transport-descriptor"></a>

<a id="sharedmemtransportdescriptor"></a>

# SharedMemTransportDescriptor

### struct test_SharedMemTransportDescriptor : public eprosima::fastdds::rtps::[SharedMemTransportDescriptor](shared_mem_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps28SharedMemTransportDescriptorE)

Shared memory transport configuration 

### Public Functions

### virtual [TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) \*create_transport() const override

Factory method pattern. It will create and return a [TransportInterface](../transport_generic_interfaces/transport_interface.md#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) corresponding to this descriptor. This provides an interface to the NetworkFactory to create the transports without the need to know about their type
