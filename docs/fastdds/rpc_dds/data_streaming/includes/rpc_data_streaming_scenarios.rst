.. _rpc_data_streaming_scenarios:

Data Streaming scenarios
^^^^^^^^^^^^^^^^^^^^^^^^

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
  back to the client, typically after receiving all the client's messages.

  This is useful when the client needs to send a large amount of data to the server,
  that cannot be sent in a single request message.

  The collection of request messages associated to the same reply is referred as *input data feed*.
  An input data feed is opened when the client calls the operation method, and closed by the user through the
  *Fast DDS-Gen* generated API when all input data has been sent.
  Users can access the client's input data at the server side implementation of the interface operations
  by reading the input feed using the generated API.

* **Bidirectional streaming**: The client sends multiple requests to the server through an input data feed,
  and the server responds with multiple replies through an output data feed.

  As both input and output data feeds are independent from each other,
  user can customize the server behaviour in the server side's operations implementation; the server could wait
  until receiving all the client's input data (*i.e* until being notified that the input feed is closed)
  before sending any reply message (*i.e* before opening the output feed), or could implement a kind of "ping-pong"
  operation, sending a message back to the client after receiving each request message.

For more information about how to define an IDL interface for data streaming and further details about handling input
and output data feeds, please refer to :ref:`rpc_data_streaming_interfaces` and
:ref:`fastddsgen_interfaces_data_streaming`.
For more information about how to build data streaming applications using the *Fast DDS-Gen* tool,
see :ref:`fastddsgen_rpc_calculator_feed_app_intro`.
