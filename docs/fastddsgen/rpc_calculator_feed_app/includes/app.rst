Generate the source code for the application
--------------------------------------------

Now that the interface source code has been generated, the next step is to generate the application source code.
All the following files should be created in the *workspace_CalculatorFeed/src* directory.

CLI and logging related files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following files are required to implement the CLI and logging functionalities of the application, and they are not
directly related to the RPC functionality.

First, create *Application.hpp* and *Application.cpp* files, which contains the abstract Application class and a factory
method to create both client and server applications, respectively. Copy the following code into the files:

* Application.hpp


..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/Application.hpp
    :language: cpp

* Application.cpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/Application.cpp
    :language: cpp


Now, create a *CLIParser.hpp* file, which contains the ``CLIParser`` class that implements the command line parser.
Copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/CLIParser.hpp
    :language: cpp

After that, create a *app_utils.hpp* file, which contains some logging utilities for our applications.
Copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/app_utils.hpp
    :language: cpp

Now, create a *InputFeedProcessor.hpp* file, which contains a class used to read the input data
from the terminal in operations with a ``@feed`` annotated input parameter.
Copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/InputFeedProcessor.hpp
    :language: cpp

Finally, create a *main.cpp* file, which contains the main function of the application, and allows the user to
close it by sending a signal. Copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/main.cpp
    :language: cpp


Server application
^^^^^^^^^^^^^^^^^^

The server application that we will create is exactly the same as the one created in the basic example.
Thus, copy the code provided in :ref:`fastddsgen_rpc_calculator_basic_server_application`
into the *ServerApp.hpp* and *ServerApp.cpp* files.

Client application
^^^^^^^^^^^^^^^^^^

The client application extends the functionality of the basic example by adding the new operations
defined in the IDL file. First, create both *ClientApp.hpp* and *ClientApp.cpp* files and copy
the following code:

* ClientApp.hpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/ClientApp.hpp
    :language: cpp

* ClientApp.cpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/clean_files/ClientApp.cpp
    :language: cpp

Examining the code
""""""""""""""""""

ClientApp.hpp
*************

The *ClientApp.hpp* file extends the code of the basic example by adding the new operations
defined in the IDL file, following the same inheritance schema:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--FEED_OPERATIONS
    :end-before: //!--

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

For output feed operations, each time that ``Operation::execute()`` is called, the client will process the data
of only one reply, so multiple calls will be required to read all the results. To address this, the
``Operation::execute()`` method will return an enum ``OperationStatus::PENDING`` when a new output feed value
is read, indicating that the output feed has not been closed yet and that client should process more data.
When the output feed is closed, the ``Operation::execute()`` method will return an enum
``OperationStatus::SUCCESS``, indicating that the operation has successfully read each of the output feed values:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--OPERATION_STATUS
    :end-before: //!--

A more detailed description of each operation execution is provided in the following subsection.

ClientApp.cpp
*************

The *ClientApp.cpp* file extends the code of the basic example by adding the new operations:

* ``FibonacciSeq`` stores the value of the ``--fibonacci`` CLI parameter using a ``n_results_`` member.
  The request is sent by calling the ``client_->fibonacci_seq()`` method, which returns an
  ``RpcClientReader`` object. The client will read the results using the ``RpcClientReader::read()`` method
  in each ``FibonacciSeq::execute()`` call, until the output feed is closed by the server.

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.cpp
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

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.cpp
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

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--ACCUMULATOR
    :end-before: //!--

* For the ``Filter`` operation, both ``RpcClientReader`` and ``RpcClientWriter`` objects are created
  by calling the ``client_->filter()`` method. First, the client will send to the server all the input feed
  data values and the filter kind using the ``RpcClientWriter::write()`` method.
  Then, each result is processed in a ``Filter::execute()`` call, until the output feed is closed by the server.
  Notice that, due to the fact that ``filter_kind_ == 2`` case is not implemented in the server side, executing this operation
  will cause an operation error.

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--FILTER
    :end-before: //!--

``ClientApp::set_operation()`` has also been extended to include the new operations:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--SET_OPERATION
    :end-before: //!--

Finally, a minimal change is required in the ``ClientApp::run()`` method to address the feed processing
loop:

..  literalinclude:: /../code/Examples/C++/RPCClientServerFeed/src/ClientApp.cpp
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
    :lines: 45-50

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

    ./build/calculator server

* In the second terminal, run the client application and specify the operation to be performed.
  For example, to perform an addition of two numbers, run the following command:

.. code-block:: bash

    ./build/calculator client --fibonacci 4

You should see the result of the operation printed on the screen:

.. code-block:: shell-session

    2025-05-08T15:05:58.574 [INFO] [ClientApp] Fibonacci sequence value: 1
    2025-05-08T15:05:59.574 [INFO] [ClientApp] Fibonacci sequence value: 1
    2025-05-08T15:06:00.574 [INFO] [ClientApp] Fibonacci sequence value: 2
    2025-05-08T15:06:01.574 [INFO] [ClientApp] Fibonacci sequence value: 3
    2025-05-08T15:06:02.574 [INFO] [ClientApp] Fibonacci sequence feed finished
    2025-05-08T15:06:02.574 [INFO] [ClientApp] Operation finished. Stopping client execution...
    2025-05-08T15:06:02.574 [INFO] [ClientApp] Client execution stopped


You can check all the available CLI options showing the help:

.. code-block:: bash

    ./build/calculator -h

The output of the rest operations should be similar to the following:

* SumAll:

.. code-block:: shell-session

    ./build/calculator client --sum-all

        Input feed help:
      - Enter a number to process it.
      - Press Enter without typing anything to close the input feed.
    2
    2025-05-08T14:59:49.835 [INFO] [ClientApp] Input sent: 2
    5
    2025-05-08T14:59:51.300 [INFO] [ClientApp] Input sent: 5
    -3
    2025-05-08T14:59:54.140 [INFO] [ClientApp] Input sent: -3

    2025-05-08T14:59:55.171 [INFO] [ClientApp] Input feed closed.
    2025-05-08T14:59:55.173 [INFO] [ClientApp] Sum result: 4
    2025-05-08T14:59:55.173 [INFO] [ClientApp] Operation finished. Stopping client execution...
    2025-05-08T14:59:55.173 [INFO] [ClientApp] Client execution stopped


* Accumulator:

.. code-block:: shell-session

    ./build/calculator client --accumulator

    Input feed help:
  - Enter a number to process it.
  - Press Enter without typing anything to close the input feed.
    2
    2025-05-08T15:01:43.786 [INFO] [ClientApp] Input sent: 2
    2025-05-08T15:01:43.787 [INFO] [ClientApp] Accumulated sum: 2
    5
    2025-05-08T15:01:45.074 [INFO] [ClientApp] Input sent: 5
    2025-05-08T15:01:45.075 [INFO] [ClientApp] Accumulated sum: 7
    -32
    2025-05-08T15:01:49.394 [INFO] [ClientApp] Input sent: -32
    2025-05-08T15:01:49.395 [INFO] [ClientApp] Accumulated sum: -25

    2025-05-08T15:01:50.489 [INFO] [ClientApp] Input feed closed.
    2025-05-08T15:01:50.491 [INFO] [ClientApp] Accumulator feed finished
    2025-05-08T15:01:50.491 [INFO] [ClientApp] Operation finished. Stopping client execution...
    2025-05-08T15:01:50.491 [INFO] [ClientApp] Client execution stopped

* Filter:

.. code-block:: shell-session

    ./build/calculator client --filter 0

    Input feed help:
  - Enter a number to process it.
  - Press Enter without typing anything to close the input feed.
    1
    2025-05-08T15:03:32.704 [INFO] [ClientApp] Input sent: 1
    3
    2025-05-08T15:03:34.072 [INFO] [ClientApp] Input sent: 3
    2
    2025-05-08T15:03:34.656 [INFO] [ClientApp] Input sent: 2
    4
    2025-05-08T15:03:36.480 [INFO] [ClientApp] Input sent: 4
    6
    2025-05-08T15:03:37.736 [INFO] [ClientApp] Input sent: 6

    2025-05-08T15:03:39.280 [INFO] [ClientApp] Input feed closed.
    2025-05-08T15:03:39.280 [INFO] [ClientApp] Filtered sequence value: 2
    2025-05-08T15:03:40.280 [INFO] [ClientApp] Filtered sequence value: 4
    2025-05-08T15:03:41.280 [INFO] [ClientApp] Filtered sequence value: 6
    2025-05-08T15:03:42.281 [INFO] [ClientApp] Filtered sequence feed finished
    2025-05-08T15:03:42.281 [INFO] [ClientApp] Operation finished. Stopping client execution...
    2025-05-08T15:03:42.281 [INFO] [ClientApp] Client execution stopped
