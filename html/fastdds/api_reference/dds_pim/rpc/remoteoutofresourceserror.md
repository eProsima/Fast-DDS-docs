<a id="api-pim-remoteoutofresourceserror"></a>

<a id="remoteoutofresourceserror-pro"></a>

# RemoteOutOfResourcesError **[Pro]**

### class RemoteOutOfResourcesError : public eprosima::fastdds::dds::rpc::[RpcRemoteException](rpcremoteexception.md#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteExceptionE)

 **[Pro]**  Exception thrown by the RPC API when the server reports out of resources. 

### Public Functions

### inline RemoteOutOfResourcesError()

Constructor. 

### inline RemoteOutOfResourcesError(const char \*msg)

Constructor with custom message.

* **Parameters:**
  **msg** – The exception message. 

### RemoteOutOfResourcesError(const [RemoteOutOfResourcesError](#_CPPv4N8eprosima7fastdds3dds3rpc25RemoteOutOfResourcesError25RemoteOutOfResourcesErrorERK25RemoteOutOfResourcesError) &other) noexcept = default

Copy constructor. 

### [RemoteOutOfResourcesError](#_CPPv4N8eprosima7fastdds3dds3rpc25RemoteOutOfResourcesErrorE) &operator=(const [RemoteOutOfResourcesError](#_CPPv4N8eprosima7fastdds3dds3rpc25RemoteOutOfResourcesErrorE) &other) noexcept = default

Copy assignment. 

### virtual ~RemoteOutOfResourcesError() noexcept = default

Destructor.
