.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fastddsgen_rpc_server_schedule_intro:

Customizing RPC Server request scheduling
=========================================

When processing an interface inside an IDL file, *Fast DDS-Gen* generates two method overloads for the creation of a
server.

The first overload creates a server with a request scheduling based on a thread pool.
Each request will be processed in a separate thread.
The number of threads in the pool is specified in the ``thread_pool_size`` argument of the method.

The second overload allows the user to inject a custom scheduling strategy for the created server.
This is done by creating a custom class implementing the |RpcServerSchedulingStrategy-api| interface.
A shared pointer to the custom class is then passed in the ``scheduler`` argument of the method.

Special care must be taken when implementing a custom scheduling strategy.
Calls to |RpcServerSchedulingStrategy::schedule_request-api| will be performed from the thread executing the server's
|RpcServer::run-api| method.
This means that incoming messages will not be processed until the execution of
|RpcServerSchedulingStrategy::schedule_request-api| finishes.
This becomes particularly important for operations that have input feed parameters, since values for input feeds will
not be processed while inside that method.

Example
"""""""

The following code snippet shows two examples of custom RPC server scheduling strategies:

.. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //!--RPC_CUSTOM_SCHEDULING_EXAMPLES
      :end-before: //!--
      :dedent: 4
