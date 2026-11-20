<a id="api-pim-remoteinvalidargumenterror"></a>

<a id="remoteinvalidargumenterror-pro"></a>

# RemoteInvalidArgumentError **[Pro]**

### class RemoteInvalidArgumentError : public eprosima::fastdds::dds::rpc::[RpcRemoteException](rpcremoteexception.md#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteExceptionE)

 **[Pro]**  Exception thrown by the RPC API when the server reports invalid arguments. 

### Public Functions

### inline RemoteInvalidArgumentError()

Constructor. 

### inline RemoteInvalidArgumentError(const char \*msg)

Constructor with custom message.

* **Parameters:**
  **msg** – The exception message. 

### RemoteInvalidArgumentError(const [RemoteInvalidArgumentError](#_CPPv4N8eprosima7fastdds3dds3rpc26RemoteInvalidArgumentError26RemoteInvalidArgumentErrorERK26RemoteInvalidArgumentError) &other) noexcept = default

Copy constructor. 

### [RemoteInvalidArgumentError](#_CPPv4N8eprosima7fastdds3dds3rpc26RemoteInvalidArgumentErrorE) &operator=(const [RemoteInvalidArgumentError](#_CPPv4N8eprosima7fastdds3dds3rpc26RemoteInvalidArgumentErrorE) &other) noexcept = default

Copy assignment. 

### virtual ~RemoteInvalidArgumentError() noexcept = default

Destructor.
