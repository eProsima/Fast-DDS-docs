<a id="api-pim-remoteunknownoperationerror"></a>

<a id="remoteunknownoperationerror-pro"></a>

# RemoteUnknownOperationError **[Pro]**

### class RemoteUnknownOperationError : public eprosima::fastdds::dds::rpc::[RpcRemoteException](rpcremoteexception.md#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteExceptionE)

 **[Pro]**  Exception thrown by the RPC API when the server does not recognize the invoked operation. 

### Public Functions

### inline RemoteUnknownOperationError()

Constructor. 

### inline RemoteUnknownOperationError(const char \*msg)

Constructor with custom message.

* **Parameters:**
  **msg** – The exception message. 

### RemoteUnknownOperationError(const [RemoteUnknownOperationError](#_CPPv4N8eprosima7fastdds3dds3rpc27RemoteUnknownOperationError27RemoteUnknownOperationErrorERK27RemoteUnknownOperationError) &other) noexcept = default

Copy constructor. 

### [RemoteUnknownOperationError](#_CPPv4N8eprosima7fastdds3dds3rpc27RemoteUnknownOperationErrorE) &operator=(const [RemoteUnknownOperationError](#_CPPv4N8eprosima7fastdds3dds3rpc27RemoteUnknownOperationErrorE) &other) noexcept = default

Copy assignment. 

### virtual ~RemoteUnknownOperationError() noexcept = default

Destructor.
