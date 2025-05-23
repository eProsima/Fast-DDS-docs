.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _rpc_data_streaming_intro:

RPC Data Streaming
------------------

*Fast DDS-Gen* tool also supports data streaming for *function-call based API*'s operations
in both client and server sides. In this case, the mapping between high-level and low-level API's
is as follows:

* On client side, each operation's input data value corresponds to a Requester's request sample in the low-level API.
  Thus, the input data streaming consists of a collection of request samples sent by the Requester to the Replier,
  representing the Client's input data that the server needs to process to compute the result of the operation.
* On the Server side, each operation's output data value corresponds to a Replier's reply sample in the low-level API.
  Thus, the output data streaming consists of a collection of reply samples sent by the Replier to the Requester,
  representing the result of the operation that the server sends back to the client.


.. include:: includes/rpc_data_streaming_scenarios.rst
.. include:: includes/rpc_data_streaming_interfaces.rst
