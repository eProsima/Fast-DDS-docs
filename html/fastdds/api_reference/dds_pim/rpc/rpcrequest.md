<a id="api-pim-rpcrequest"></a>

<a id="rpcrequest-pro"></a>

# RpcRequest **[Pro]**

### class RpcRequest

 **[Pro]**  An interface with generic RPC requests functionality. 

### Public Functions

### virtual ~RpcRequest() noexcept = default

Destructor. 

### virtual const eprosima::fastdds::rtps::[GUID_t](../../rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) &get_client_id() const = 0

Get the GUID of the client that made the request. 

* **Returns:**
  The GUID of the client that made the request. 

### virtual const eprosima::fastdds::rtps::[RemoteLocatorList](../../rtps/common/RemoteLocators/RemoteLocatorList.md#_CPPv4N8eprosima7fastdds4rtps17RemoteLocatorListE) &get_client_locators() const = 0

Get the locators of the client that made the request. 

* **Returns:**
  The locators of the client that made the request.
