<a id="api-pim-rpcoperationerror"></a>

<a id="rpcoperationerror-pro"></a>

# RpcOperationError **[Pro]**

### class RpcOperationError : public eprosima::fastdds::dds::rpc::[RpcException](rpcexception.md#_CPPv4N8eprosima7fastdds3dds3rpc12RpcExceptionE)

 **[Pro]**  Base class for exceptions thrown by the RPC API when the server communicates an error. 

### Public Functions

### inline RpcOperationError(const std::string &message)

Constructor. 

### inline RpcOperationError(const char \*message)

Constructor. 

### RpcOperationError(const [RpcOperationError](#_CPPv4N8eprosima7fastdds3dds3rpc17RpcOperationError17RpcOperationErrorERK17RpcOperationError) &other) noexcept = default

Copy constructor. 

### [RpcOperationError](#_CPPv4N8eprosima7fastdds3dds3rpc17RpcOperationErrorE) &operator=(const [RpcOperationError](#_CPPv4N8eprosima7fastdds3dds3rpc17RpcOperationErrorE) &other) noexcept = default

Copy assignment. 

### virtual ~RpcOperationError() noexcept = default

Destructor.
