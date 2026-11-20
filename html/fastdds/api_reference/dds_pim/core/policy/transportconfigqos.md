<a id="api-pim-transportconfigqos"></a>

<a id="transportconfigqos"></a>

# TransportConfigQos

### class TransportConfigQos : public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Qos Policy to configure the transport layer. 

### Public Functions

### inline TransportConfigQos()

Constructor. 

### virtual ~TransportConfigQos() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### std::vector<std::shared_ptr<fastdds::rtps::[TransportDescriptorInterface](../../../transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE)>> user_transports

User defined transports to use alongside or in place of builtins. 

### bool use_builtin_transports

Set as false to disable the default UDPv4 implementation.

By default, true.

### std::uint32_t send_socket_buffer_size

Send socket buffer size for the send resource. 

Zero value indicates to use default system buffer size.

By default, 0.

### std::uint32_t listen_socket_buffer_size

Listen socket buffer for all listen resources. 

Zero value indicates to use default system buffer size.

By default, 0.

### rtps::[ThreadSettings](../../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) builtin_transports_reception_threads_

Thread settings for the builtin transports reception threads. 

### std::uint32_t max_msg_size_no_frag

Maximum message size used to avoid fragmentation, set ONLY in LARGE_DATA. 

If this value is not zero, the network factory will allow the initialization of UDP transports with maxMessageSize higher than 65500K. 

### fastdds::rtps::NetmaskFilterKind netmask_filter

Netmask filter configuration.
