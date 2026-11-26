<a id="api-pim-rpcremoteexception"></a>

<a id="rpcremoteexception-pro"></a>

# RpcRemoteException **[Pro]**

### class RpcRemoteException : public eprosima::fastdds::dds::rpc::[RpcException](rpcexception.md#_CPPv4N8eprosima7fastdds3dds3rpc12RpcExceptionE)

 **[Pro]**  Base class for exceptions that map to a RpcExceptionCode_t. 

Subclassed by [eprosima::fastdds::dds::rpc::RemoteInvalidArgumentError](remoteinvalidargumenterror.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RemoteInvalidArgumentError), [eprosima::fastdds::dds::rpc::RemoteOutOfResourcesError](remoteoutofresourceserror.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RemoteOutOfResourcesError), [eprosima::fastdds::dds::rpc::RemoteUnknownExceptionError](remoteunknownexceptionerror.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RemoteUnknownExceptionError), [eprosima::fastdds::dds::rpc::RemoteUnknownOperationError](remoteunknownoperationerror.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RemoteUnknownOperationError), [eprosima::fastdds::dds::rpc::RemoteUnsupportedError](remoteunsupportederror.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1RemoteUnsupportedError)

### Public Functions

### inline RpcRemoteException([RemoteExceptionCode_t](remoteexceptioncode_t.md#_CPPv4N8eprosima7fastdds3dds3rpc21RemoteExceptionCode_tE) code, const char \*msg)

Constructor. 

### RpcRemoteException(const [RpcRemoteException](#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteException18RpcRemoteExceptionERK18RpcRemoteException) &other) noexcept = default

Copy constructor. 

### [RpcRemoteException](#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteExceptionE) &operator=(const [RpcRemoteException](#_CPPv4N8eprosima7fastdds3dds3rpc18RpcRemoteExceptionE) &other) noexcept = default

Copy assignment. 

### virtual ~RpcRemoteException() noexcept = default

Destructor. 

### inline [RemoteExceptionCode_t](remoteexceptioncode_t.md#_CPPv4N8eprosima7fastdds3dds3rpc21RemoteExceptionCode_tE) code() const noexcept

Get the exception code.

* **Returns:**
  The exception code.
