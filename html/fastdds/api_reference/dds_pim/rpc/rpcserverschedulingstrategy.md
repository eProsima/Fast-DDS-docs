<a id="api-pim-rpcserverschedulingstrategy"></a>

<a id="rpcserverschedulingstrategy-pro"></a>

# RpcServerSchedulingStrategy **[Pro]**

### class RpcServerSchedulingStrategy

 **[Pro]**  An interface with generic RPC requests functionality. 

### Public Functions

### virtual ~RpcServerSchedulingStrategy() noexcept = default

Destructor. 

### virtual void schedule_request(const std::shared_ptr<[RpcRequest](rpcrequest.md#_CPPv4N8eprosima7fastdds3dds3rpc10RpcRequestE)> &request, const std::shared_ptr<[RpcServer](rpcserver.md#_CPPv4N8eprosima7fastdds3dds3rpc9RpcServerE)> &server) = 0

Schedule a request for processing. 

This method is called when a request is received and should be processed by the server. The implementation should decide how to handle the request, whether to process it immediately, or to queue it for later processing.

A call to server->execute_request(request) should eventually be made to process the request.

* **Parameters:**
  * **request** – The request to schedule. 
  * **server** – The server instance that should process the request. 

### virtual void server_stopped(const std::shared_ptr<[RpcServer](rpcserver.md#_CPPv4N8eprosima7fastdds3dds3rpc9RpcServerE)> &server) = 0

Informs that a server has been stopped and all its requests have been cancelled. 

* **Parameters:**
  **server** – The server instance that has been stopped.
