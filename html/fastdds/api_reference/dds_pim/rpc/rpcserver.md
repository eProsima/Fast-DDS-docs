<a id="api-pim-rpcserver"></a>

<a id="rpcserver-pro"></a>

# RpcServer **[Pro]**

### class RpcServer

 **[Pro]**  An interface with generic RPC server functionality. 

### Public Functions

### virtual ~RpcServer() noexcept = default

Destructor. 

### virtual void run() = 0

Run the server. 

This method starts the server and begins processing requests. The method will block until the server is stopped. 

### virtual void stop() = 0

Stop the server. 

This method stops the server and releases all resources. It will cancel all pending requests, and wait for all processing threads to finish before returning. 

### virtual void execute_request(const std::shared_ptr<[RpcRequest](rpcrequest.md#_CPPv4N8eprosima7fastdds3dds3rpc10RpcRequestE)> &request) = 0

Perform execution of a client request. 

* **Parameters:**
  **request** – The client request to execute.
