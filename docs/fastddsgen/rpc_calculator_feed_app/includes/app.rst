Generate the source code for the application
--------------------------------------------

Now that the interface source code has been generated, the next step is to generate the application source code.
All the following files should be created in the *workspace_CalculatorFeed/src* directory.

Server application
^^^^^^^^^^^^^^^^^^

The server application that we will create is exactly the same as the one created in the basic example.
Thus, copy the code provided in :ref:`fastddsgen_rpc_calculator_basic_server_application`
into the ``CalculatorServer.cpp`` file.

Client application
^^^^^^^^^^^^^^^^^^

The client application extends the functionality of the basic example by adding the new operations
defined in the IDL file. Create a ``CalculatorClient.cpp`` file with the following content:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp

Examining the code
""""""""""""""""""

The ``CalculatorClient.cpp`` file extends the code of the basic example by adding the new operations
defined in the IDL file, following the same inheritance schema.

Notice the following facts:

* The operations that expect an output feed (*i.e* ``FibonacciSeq``, ``Accumulator`` and ``Filter`` operations)
  store internally an ``RpcClientReader`` reference. As the client expects multiple replies from the server for
  the same request, it will use the ``RpcClientReader::read()`` method to read the results until the output feed
  is closed by the server.

* The operations that expect an input feed for some of their parameters
  (*i.e* ``SumAll``, ``Accumulator`` and ``Filter`` operations) store internally an
  ``RpcClientWriter`` reference. As the client will send multiple values to the server, it will use the
  ``RpcClientWriter::write()`` method to send the values and the ``RpcClientWriter::finish()`` method to notify
  the server that the input feed is finished.

For input feed operations, a simple ``InputFeedProcessor`` class is used to parse the input user data from terminal.
It allows the user to send a new number or close the input feed by accepting the dialog with empty data:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--INPUT_FEED_PROCESSOR
    :end-before: //!--

For output feed operations, each time that ``Operation::execute()`` is called, the client will process the data
of only one reply, so multiple calls will be required to read all the results. To address this, the
``Operation::execute()`` method will return an enum ``OperationStatus::PENDING`` when a new output feed value
is read, indicating that the output feed has not been closed yet and that client should process more data.
When the output feed is closed, the ``Operation::execute()`` method will return an enum
``OperationStatus::SUCCESS``, indicating that the operation has successfully read each of the output feed values:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--OPERATION_STATUS
    :end-before: //!--

A more detailed description of each operation execution is provided below:

* ``FibonacciSeq`` stores the number of requested numbers using using a ``n_results_`` member.
  The request is sent by calling the ``client_->fibonacci_seq()`` method, which returns an
  ``RpcClientReader`` object. The client will read the results using the ``RpcClientReader::read()`` method
  in each ``FibonacciSeq::execute()`` call, until the output feed is closed by the server.

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--FIBONACCI_SEQ
    :end-before: //!--

* In the first call of the ``SumAll::execute()`` method, the client will create an
  ``RpcClientWriter`` object by calling the ``client_->sum_all()`` method and passing it as an output parameter.
  Then, the input feed is parsed from the user terminal using the ``InputFeedProcessor`` class, sending each
  input data value to the server using the ``RpcClientWriter::write()`` method. When the user finish the input feed
  (by accepting the terminal dialog with an empty value),
  the client closes the input feed using the ``RpcClientWriter::finish()``
  and waits for the reply. When the reply is received,
  the result is stored in ``result_`` member and printed on the screen.

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--SUM_ALL
    :end-before: //!--

* For the ``Accumulator`` operation, both ``RpcClientReader`` and ``RpcClientWriter`` objects are created
  by calling the ``client_->accumulator()`` method. The client will read the results using the
  ``RpcClientReader::read()`` method in each ``Accumulator::execute()`` call, until the output feed is closed
  by the server. The input feed is parsed from the user terminal using the ``InputFeedProcessor`` class,
  sending each input data value to the server using the ``RpcClientWriter::write()`` method.
  When the user finish the input feed (by accepting the terminal dialog with an empty value),
  the client closes the input feed using the ``RpcClientWriter::finish()``
  and waits for the reply.
  Each time that ``Accumulator::execute()`` is called, the client sends a new input value to the server
  and waits for the accumulated sum result, until the input and output feeds are closed.

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--ACCUMULATOR
    :end-before: //!--

* For the ``Filter`` operation, both ``RpcClientReader`` and ``RpcClientWriter`` objects are created
  by calling the ``client_->filter()`` method. First, the client will send to the server all the input feed
  data values and the filter kind using the ``RpcClientWriter::write()`` method.
  Then, each result is processed in a ``Filter::execute()`` call, until the output feed is closed by the server.
  To simplify the input parsing, the filter is fixed to be ``FilterKind::EVEN``, *i.e* the input feed is filtered
  to only return even numbers:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--FILTER
    :end-before: //!--

``ClientApp::set_operation()`` has also been extended to include the new operations:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--SET_OPERATION
    :end-before: //!--

Finally, a minimal change is required in the ``send_request()`` method to handle the feed operations:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--FEED_LOOP
    :end-before: //!--

Note that this change does not modify the behavior of the non-feed operations, as they never return
``OperationStatus::PENDING`` status.

Update the *CMakeLists.txt* file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before building the application, we need to update the *CMakeLists.txt* file of the workspace
to add the executable and the required libraries. The following code should be added at the end
of the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/CMakeLists.txt
    :language: cmake
    :lines: 44-56

Build the application
---------------------

Now that all the files are created, the application can be built.
To do so, open a terminal in the *workspace_CalculatorFeed* directory and run the following commands:

.. code-block:: bash

    mkdir build && cd build
    cmake ..
    cmake --build .

The generated executable will be located in the *build* subdirectory.

Run the application
-------------------

To test the application, open two terminals in the *workspace_CalculatorFeed* directory
and execute the following commands:

* In the first terminal, run the server application:

.. code-block:: bash

    ./build/feed_server

* In the second terminal, run the client application and specify the operation to be performed.
  For example, to get the first five numbers of the Fibonacci sequence, run the following command:

.. code-block:: bash

    ./build/feed_client fib

You should see the result of the operation printed on the screen:

.. code-block:: shell-session

    Attempting to send request, attempt 1/10
    Configuring FIBONACCI operation with 5 results
    Fibonacci sequence value: 1
    Fibonacci sequence value: 1
    Fibonacci sequence value: 2
    Fibonacci sequence value: 3
    Fibonacci sequence value: 5
    Fibonacci sequence feed finished
    Request sent successfully

The output of the rest operations should be similar to the following:

* SumAll:

.. code-block:: shell-session

    ./build/feed_client sumall

    Configuring SUM_ALL operation
    Input feed help:
      - Enter a number to process it.
      - Press Enter without typing anything to close the input feed.
    2
    Input sent: 2
    3
    Input sent: 3
    -32
    Input sent: -32
    -25
    Input sent: -25
    -12
    Input sent: -12

    Input feed closed.
    Sum result: -64
    Request sent successfully


* Accumulator:

.. code-block:: shell-session

    ./build/feed_client acc

    Configuring ACCUMULATOR operation
    Input feed help:
      - Enter a number to process it.
      - Press Enter without typing anything to close the input feed.
    16
    Input sent: 16
    Accumulated sum: 16
    32
    Input sent: 32
    Accumulated sum: 48
    -13
    Input sent: -13
    Accumulated sum: 35
    -22
    Input sent: -22
    Accumulated sum: 13
    -1
    Input sent: -1
    Accumulated sum: 12

    Input feed closed.
    Accumulator feed finished
    Request sent successfully

* Filter:

.. code-block:: shell-session

    ./build/feed_client filter

    Configuring FILTER operation for even numbers
    Input feed help:
      - Enter a number to process it.
      - Press Enter without typing anything to close the input feed.
    2
    Input sent: 2
    11
    Input sent: 11
    32
    Input sent: 32
    15
    Input sent: 15
    -23
    Input sent: -23
    -15
    Input sent: -15
    4
    Input sent: 4

    Input feed closed.
    Filtered sequence value: 2
    Filtered sequence value: 32
    Filtered sequence value: 4
    Filtered sequence feed finished
    Request sent successfully
