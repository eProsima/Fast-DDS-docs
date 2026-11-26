<a id="python-api-pim-transportconfigqos"></a>

<a id="transportconfigqos"></a>

# TransportConfigQos

### *class* fastdds.TransportConfigQos

Qos Policy to configure the transport layer

#### *property* builtin_transports_reception_threads_

Thread settings for the builtin transports reception threads

#### clear()

Clears the QosPolicy object

#### *property* listen_socket_buffer_size

> Listen socket buffer for all listen resources. Zero value indicates to use default system buffer size.

By default, 0.

#### *property* max_msg_size_no_frag

Maximum message size used to avoid fragmentation, set ONLY in LARGE_DATA.

If this value is not zero, the network factory will allow the initialization of UDP transports with maxMessageSize
higher than 65500K.

#### *property* netmask_filter

Netmask filter configuration

#### *property* send_socket_buffer_size

Send socket buffer size for the send resource. Zero value indicates to use default system buffer size.

By default, 0.

#### *property* thisown

The membership flag

#### *property* use_builtin_transports

Set as false to disable the default UDPv4 implementation. 
: By default, true.

#### *property* user_transports

User defined transports to use alongside or in place of builtins.
