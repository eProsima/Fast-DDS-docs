<a id="api-pim-rpcfeedcancelledexception"></a>

<a id="rpcfeedcancelledexception-pro"></a>

# RpcFeedCancelledException **[Pro]**

### class RpcFeedCancelledException : public eprosima::fastdds::dds::rpc::[RpcException](rpcexception.md#_CPPv4N8eprosima7fastdds3dds3rpc12RpcExceptionE)

 **[Pro]**  Exception thrown by the RPC API when the client cancels an input feed. 

### Public Functions

### inline RpcFeedCancelledException([RpcStatusCode](rpcstatuscode.md#_CPPv4N8eprosima7fastdds3dds3rpc13RpcStatusCodeE) reason)

Constructor. 

### RpcFeedCancelledException(const [RpcFeedCancelledException](#_CPPv4N8eprosima7fastdds3dds3rpc25RpcFeedCancelledException25RpcFeedCancelledExceptionERK25RpcFeedCancelledException) &other) noexcept = default

Copy constructor. 

### [RpcFeedCancelledException](#_CPPv4N8eprosima7fastdds3dds3rpc25RpcFeedCancelledExceptionE) &operator=(const [RpcFeedCancelledException](#_CPPv4N8eprosima7fastdds3dds3rpc25RpcFeedCancelledExceptionE) &other) noexcept = default

Copy assignment. 

### virtual ~RpcFeedCancelledException() noexcept = default

Destructor.
