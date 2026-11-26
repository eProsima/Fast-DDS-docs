<a id="api-pim-remoteunknownexceptionerror"></a>

<a id="remoteunknownexceptionerror-pro"></a>

# RemoteUnknownExceptionError **[Pro]**

### class RemoteUnknownExceptionError : public eprosima::fastdds::dds::rpc::[RpcRemoteException](rpcremoteexception.md#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteExceptionE)

 **[Pro]**  Exception thrown by the RPC API when the service implementation raises an unspecified exception. 

### Public Functions

### inline RemoteUnknownExceptionError()

Constructor. 

### inline RemoteUnknownExceptionError(const char \*msg)

Constructor with custom message.

* **Parameters:**
  **msg** – The exception message. 

### RemoteUnknownExceptionError(const [RemoteUnknownExceptionError](#_CPPv4N8eprosima7fastdds3dds3rpc27RemoteUnknownExceptionError27RemoteUnknownExceptionErrorERK27RemoteUnknownExceptionError) &other) noexcept = default

Copy constructor. 

### [RemoteUnknownExceptionError](#_CPPv4N8eprosima7fastdds3dds3rpc27RemoteUnknownExceptionErrorE) &operator=(const [RemoteUnknownExceptionError](#_CPPv4N8eprosima7fastdds3dds3rpc27RemoteUnknownExceptionErrorE) &other) noexcept = default

Copy assignment. 

### virtual ~RemoteUnknownExceptionError() noexcept = default

Destructor.
