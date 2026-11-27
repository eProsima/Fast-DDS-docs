<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="fastddsgen-rpc-server-schedule-intro"></a>

<a id="customizing-rpc-server-request-scheduling"></a>

# Customizing RPC Server request scheduling

When processing an interface inside an IDL file, *Fast DDS-Gen* generates two method overloads for the creation of a
server.

The first overload creates a server with a request scheduling based on a thread pool.
Each request will be processed in a separate thread.
The number of threads in the pool is specified in the `thread_pool_size` argument of the method.

The second overload allows the user to inject a custom scheduling strategy for the created server.
This is done by creating a custom class implementing the [`RpcServerSchedulingStrategy`](../../fastdds/api_reference/dds_pim/rpc/rpcserverschedulingstrategy.md#_CPPv4N8eprosima7fastdds3dds3rpc27RpcServerSchedulingStrategyE) interface.
A shared pointer to the custom class is then passed in the `scheduler` argument of the method.

Special care must be taken when implementing a custom scheduling strategy.
Calls to [`schedule_request()`](../../fastdds/api_reference/dds_pim/rpc/rpcserverschedulingstrategy.md#_CPPv4N8eprosima7fastdds3dds3rpc27RpcServerSchedulingStrategy16schedule_requestERKNSt10shared_ptrI10RpcRequestEERKNSt10shared_ptrI9RpcServerEE) will be performed from the thread executing the server’s
[`run()`](../../fastdds/api_reference/dds_pim/rpc/rpcserver.md#_CPPv4N8eprosima7fastdds3dds3rpc9RpcServer3runEv) method.
This means that incoming messages will not be processed until the execution of
[`schedule_request()`](../../fastdds/api_reference/dds_pim/rpc/rpcserverschedulingstrategy.md#_CPPv4N8eprosima7fastdds3dds3rpc27RpcServerSchedulingStrategy16schedule_requestERKNSt10shared_ptrI10RpcRequestEERKNSt10shared_ptrI9RpcServerEE) finishes.
This becomes particularly important for operations that have input feed parameters, since values for input feeds will
not be processed while inside that method.

<a id="example"></a>

## Example

The following code snippet shows two examples of custom RPC server scheduling strategies:

```c++
// A scheduling strategy where requests are processed in the same thread where they are received.
// Should not be used in servers with feed operations.
struct DirectRequestScheduling : public eprosima::fastdds::dds::rpc::RpcServerSchedulingStrategy
{
    void schedule_request(
            const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcRequest>& request,
            const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
    {
        server->execute_request(request);
    }

    void server_stopped(
            const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
    {
        static_cast<void>(server);
    }

};

// A scheduling strategy where each request is processed in a detached thread.
struct DetachedThreadRequestScheduling : public eprosima::fastdds::dds::rpc::RpcServerSchedulingStrategy
{
    void schedule_request(
            const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcRequest>& request,
            const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
    {
        std::thread([server, request]()
                {
                    server->execute_request(request);
                }).detach();
    }

    void server_stopped(
            const std::shared_ptr<eprosima::fastdds::dds::rpc::RpcServer>& server) override
    {
        static_cast<void>(server);
    }

};
```
