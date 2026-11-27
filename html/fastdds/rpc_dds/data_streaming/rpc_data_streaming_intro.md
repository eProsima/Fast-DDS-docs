<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="rpc-data-streaming-intro"></a>

<a id="rpc-data-streaming"></a>

# RPC Data Streaming

*Fast DDS-Gen* tool also supports data streaming for *function-call based API*’s operations
in both client and server sides: a client can send a stream of input values for a single operation call
(input feed), and a server can send a stream of results back for a single call (output feed).

<a id="rpc-data-streaming-scenarios"></a>

<a id="data-streaming-scenarios"></a>

## Data Streaming scenarios

Three different scenarios can be considered:

* **Server side streaming**: The client sends a single request to the server, and the server sends multiple replies
  back to the client associated to the same request.

  This is useful when the server needs to send a large amount of data to the client,
  that cannot be sent in a single reply message.

  The collection of reply messages associated to the same request is referred as *output data feed*.
  An output data feed is opened when the server sends the first reply message, and closed when the server
  sends the last reply message, notifying the client that no more reply messages are expected to be received.
  Each data message can be obtained at the client side by reading this output feed using the *Fast DDS-Gen*
  generated API.
* **Client side streaming**: The client sends multiple requests to the server, and the server sends a single reply
  back to the client, typically after receiving all the client’s messages.

  This is useful when the client needs to send a large amount of data to the server,
  that cannot be sent in a single request message.

  The collection of request messages associated to the same reply is referred as *input data feed*.
  An input data feed is opened when the client calls the operation method, and closed by the user through the
  *Fast DDS-Gen* generated API when all input data has been sent.
  Users can access the client’s input data at the server side implementation of the interface operations
  by reading the input feed using the generated API.
* **Bidirectional streaming**: The client sends multiple requests to the server through an input data feed,
  and the server responds with multiple replies through an output data feed.

  As both input and output data feeds are independent from each other,
  user can customize the server behaviour in the server side’s operations implementation; the server could wait
  until receiving all the client’s input data (*i.e* until being notified that the input feed is closed)
  before sending any reply message (*i.e* before opening the output feed), or could implement a kind of “ping-pong”
  operation, sending a message back to the client after receiving each request message.

For more information about how to define an IDL interface for data streaming and further details about handling input
and output data feeds, please refer to [Data Streaming interfaces](#rpc-data-streaming-interfaces) and
[Defining data streaming operations](../../../fastddsgen/interfaces/interfaces.md#fastddsgen-interfaces-data-streaming).
For more information about how to build data streaming applications using the *Fast DDS-Gen* tool,
see [Building a RPC Client/Server application with data streaming](../../../fastddsgen/rpc_calculator_feed_app/intro.md#fastddsgen-rpc-calculator-feed-app-intro).

<a id="rpc-data-streaming-interfaces"></a>

<a id="data-streaming-interfaces"></a>

## Data Streaming interfaces

Fast DDS provides a set of interfaces to support data streaming in the RPC API,
as explained in [Data Streaming scenarios](#rpc-data-streaming-scenarios). When *Fast DDS-Gen* generates the source code,
they are automatically implemented, so the user can use them directly in an RPC application involving
data streaming without additional steps.

<a id="server-side-streaming"></a>

### Server side streaming

Output feed is managed using the `RpcServerWriter` and `RpcClientReader` interfaces,
whose implementations are provided by the *Fast DDS-Gen* generated server and client code, respectively.

User can send each computed value to the client by calling `write()`.
Closing the output feed explicitly is not necessary: it is closed automatically when the server-side
operation implementation returns.

On the client side, received values are read through the `read()` methods,
which block until a new output feed value is available, the feed is closed, or a configured timeout expires.

User can also cancel an active output feed at the client side by calling `cancel()` method.

<a id="client-side-streaming"></a>

### Client side streaming

Input feed is managed using the `RpcClientWriter` and `RpcServerReader` interfaces,
whose implementations are provided by the *Fast DDS-Gen* generated client and server code, respectively.

User can send each input data value to the server by calling `write()` method,
or close the input feed by calling `finish()` method.
Additionally, an [`RpcStatusCode`](../../api_reference/dds_pim/rpc/rpcstatuscode.md#_CPPv4N8eprosima7fastdds3dds3rpc13RpcStatusCodeE) can be specified to indicate the reason for closing the input feed.

#### NOTE
If no reason is specified in the `finish()` method, it is configured to be
`RPC_STATUS_CODE_OK` by default.

On the server side, received values are read through the `read()` methods,
which block until a new input feed value is available, the feed is closed, or a configured timeout expires.

<a id="bidirectional-streaming"></a>

### Bidirectional streaming

Bidirectional streaming uses independent input and output feeds, so they can be managed separately using the
interfaces of the previous cases.
