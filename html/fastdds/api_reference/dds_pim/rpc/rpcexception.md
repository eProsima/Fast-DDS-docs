<a id="api-pim-rpcexception"></a>

<a id="rpcexception-pro"></a>

# RpcException **[Pro]**

### class RpcException

 **[Pro]**  Base class for all exceptions thrown by the RPC API. 

Subclassed by [eprosima::fastdds::dds::rpc::RpcBrokenPipeException](rpcbrokenpipeexception.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RpcBrokenPipeException), [eprosima::fastdds::dds::rpc::RpcFeedCancelledException](rpcfeedcancelledexception.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RpcFeedCancelledException), [eprosima::fastdds::dds::rpc::RpcOperationError](rpcoperationerror.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RpcOperationError), [eprosima::fastdds::dds::rpc::RpcRemoteException](rpcremoteexception.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RpcRemoteException), [eprosima::fastdds::dds::rpc::RpcTimeoutException](rpctimeoutexception.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RpcTimeoutException)

### Public Functions

### inline explicit RpcException(const std::string &message)

Constructor.

* **Parameters:**
  **message** – The exception message. 

### inline explicit RpcException(const char \*message)

Constructor.

* **Parameters:**
  **message** – The exception message. 

### RpcException(const [RpcException](#_CPPv4N8eprosima7fastdds3dds3rpc12RpcException12RpcExceptionERK12RpcException) &other) noexcept = default

Copy constructor. 

### [RpcException](#_CPPv4N8eprosima7fastdds3dds3rpc12RpcExceptionE) &operator=(const [RpcException](#_CPPv4N8eprosima7fastdds3dds3rpc12RpcExceptionE) &other) noexcept = default

Copy assignment. 

### virtual ~RpcException() noexcept = default

Destructor. 

### inline const char \*what() const noexcept

Returns the explanatory string.
