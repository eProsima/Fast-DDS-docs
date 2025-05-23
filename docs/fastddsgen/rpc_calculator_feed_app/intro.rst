.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fastddsgen_rpc_calculator_feed_app_intro:

Building a RPC Client/Server application with data streaming
============================================================

*Fast DDS-Gen* supports the generation of source code for interfaces that specify
data streaming operations using the ``@feed`` builtin annotation.

This section extends the previously discussed example of a calculator service
(see :ref:`fastddsgen_rpc_calculator_basic_app_intro`) to include the following data streaming operations:

* **fibonacci_seq**: Returns a feed of results with the n_results first elements of the Fibonacci sequence.
* **sum_all**: Returns the sum of all the received values through a feed when it is closed.
* **accumulator**: Returns a feed of results with the sum of all received values.
* **filter**: Returns a feed of results with the received values that match an input filter.

The entire example source code is available in this
`link <https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerFeed>`_

.. include:: includes/background.rst
.. include:: includes/workspace.rst
.. include:: includes/cmake.rst
.. include:: includes/idl.rst
.. include:: includes/code_generation.rst
.. include:: includes/app.rst
