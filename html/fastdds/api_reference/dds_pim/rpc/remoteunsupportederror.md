<a id="api-pim-remoteunsupportederror"></a>

<a id="remoteunsupportederror-pro"></a>

# RemoteUnsupportedError **[Pro]**

### class RemoteUnsupportedError : public eprosima::fastdds::dds::rpc::[RpcRemoteException](rpcremoteexception.md#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteExceptionE)

 **[Pro]**  Exception thrown by the RPC API when the server reports an unsupported operation. 

### Public Functions

### inline RemoteUnsupportedError()

Constructor. 

### inline RemoteUnsupportedError(const char \*msg)

Constructor with custom message.

* **Parameters:**
  **msg** – The exception message. 

### RemoteUnsupportedError(const [RemoteUnsupportedError](#_CPPv4N8eprosima7fastdds3dds3rpc22RemoteUnsupportedError22RemoteUnsupportedErrorERK22RemoteUnsupportedError) &other) noexcept = default

Copy constructor. 

### [RemoteUnsupportedError](#_CPPv4N8eprosima7fastdds3dds3rpc22RemoteUnsupportedErrorE) &operator=(const [RemoteUnsupportedError](#_CPPv4N8eprosima7fastdds3dds3rpc22RemoteUnsupportedErrorE) &other) noexcept = default

Copy assignment. 

### virtual ~RemoteUnsupportedError() noexcept = default

Destructor.
