.. _rpc_data_streaming_interfaces:

Data Streaming interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS provides a set of interfaces to support data streaming in the RPC API,
as explained in :ref:`rpc_data_streaming_scenarios`. When *Fast DDS-Gen* generates the source code,
they are automatically implemented, so the user can use them directly in an RPC application involving
data streaming without additional steps.

Server side streaming
*********************

Output feed is managed using the |RpcServerWriter-api| and |RpcClientReader-api| interfaces,
implemented in the server and client files of the
*Fast DDS-Gen* generate code, respectively, for each operation defined in the IDL interface.
For each ``RpcServerWriter`` implementation, *Fast DDS-Gen* stores a reference to a |Replier-api| instance,
which is used to send the reply samples associated to the output feed data.

User can send each computed value to the client by calling |RpcServerWriter::write-api|.
Specifying when the output data feed is closed is not necessary, as this is done automatically (after finishing
the server-side operation execution)
by the **Fast DDS-Gen** generated code sending a last reply sample specifying that no more values will be sent.

Similarly, for each ``RpcClientReader`` implementation, *Fast DDS-Gen* stores a reference
to a |Requester-api| instance, which is used to received the reply samples associated to the output feed data.
When a new sample is received, client stores the value sent by the server internally in a queue,
which can be accessed by the user through the |RpcClientReader::read-api| methods. This operation blocks the
thread until a new output feed value is available, or a configured timeout expires.

User can also cancel an active output feed at the client side by calling |RpcClientReader::cancel-api| method.

Client side streaming
*********************

Input feed is managed using the |RpcClientWriter-api| and |RpcServerReader-api| interfaces,
implemented in the client files and server files of the
*Fast DDS-Gen* generated code, respectively, for each operation defined in the IDL interface.
For each ``RpcClientWriter`` implementation, *Fast DDS-Gen* stores a reference to a |Requester-api| instance,
which is used to send the reply samples associated to the input feed data.

User can send each input data value to the server by calling |RpcClientWriter::write-api| method,
or close the input feed by calling |RpcClientWriter::finish-api| method.
Additionally, an |RpcStatusCode-api| can be specified to indicate the reason for closing the input feed.

.. note::
    If no reason is specified in the |RpcClientWriter::finish-api| method, it is configured to be
    ``RPC_STATUS_CODE_OK`` by default.

Similarly, for each ``RpcServerReader`` implementation, *Fast DDS-Gen* stores a reference
to a |Replier-api| instance, which is used to received the request samples associated to the input feed data.
When a new sample is received, server stores the value sent by the client internally in a queue,
which can be accessed by the user through the |RpcServerReader::read-api| methods. This operation blocks the
thread until a new input feed value is available, or a configured timeout expires.

Bidirectional streaming
***********************

Bidirectional streaming uses independent input and output feeds, so they can be managed separately using the
interfaces of the previous cases.
