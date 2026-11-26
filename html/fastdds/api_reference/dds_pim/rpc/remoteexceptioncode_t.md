<a id="api-pim-remoteexceptioncode-t"></a>

<a id="remoteexceptioncode-t"></a>

# RemoteExceptionCode_t

### enum class eprosima::fastdds::dds::rpc::RemoteExceptionCode_t : std::int32_t

Enumeration of possible error codes that can be returned by a remote service. Extracted from DDS-RPC v1.0 - 7.5.2 Mapping of Error Codes. 

*Values:*

### enumerator REMOTE_EX_OK

The request was executed successfully. 

### enumerator REMOTE_EX_UNSUPPORTED

Operation is valid but it is unsupported (a.k.a. not implemented). 

### enumerator REMOTE_EX_INVALID_ARGUMENT

The value of a parameter passed has an illegal value. 

### enumerator REMOTE_EX_OUT_OF_RESOURCES

The remote service ran out of resources while processing the request. 

### enumerator REMOTE_EX_UNKNOWN_OPERATION

The operation called is unknown. 

### enumerator REMOTE_EX_UNKNOWN_EXCEPTION

A generic, unspecified exception was raised by the service implementation.
