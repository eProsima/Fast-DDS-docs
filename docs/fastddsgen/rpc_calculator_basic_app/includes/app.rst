Generate the source code for the application
--------------------------------------------

Now that the interface source code has been generated, the next step is to generate the application source code.
All the following files should be created in the ``workspace_CalculatorBasic/src`` directory.

.. _fastddsgen_rpc_calculator_basic_server_application:

Server application
^^^^^^^^^^^^^^^^^^

The first step is to create the server application. Running it, the user can run an RPC server
ready to process requests from client applications and send replies.

Create a ``CalculatorServer.cpp`` file and
copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/CalculatorServer.cpp
    :language: cpp

Examining the code
""""""""""""""""""

The ``CalculatorServer.cpp`` file contains the implementation of the ``Server`` class, formed by
a Server instance and its related DomainParticipant instance:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorServer.cpp
    :language: cpp
    :start-after: //!--SERVER_PROTECTED_MEMBERS
    :end-before: //!--

When a ``Server`` instance is initialized, both ``DomainParticipant`` and ``CalculatorServer`` instances
are created. The ``CalculatorServer`` instance is created using the previously created
``DomainParticipant`` instance, the implementation of the server-side IDL interface's operations
and the name of the RPC service. |HistoryQosPolicyKind-api| is set to |KEEP_ALL_HISTORY_QOS-api| to avoid losing
replies when the server is processing multiple requests:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorServer.cpp
    :language: cpp
    :start-after: //!--INIT
    :end-before: //!--

Once all the Server's entities are created, the RPC server is started using the
``CalculatorServer::run()`` method:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorServer.cpp
    :language: cpp
    :start-after: //!--RUN
    :end-before: //!--

Similarly, when the application is stopped, the
RPC server is also stopped using the ``CalculatorServer::stop()`` method:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorServer.cpp
    :language: cpp
    :start-after: //!--STOP
    :end-before: //!--

Before finishing the application, all the internal DDS and RPC entities involved in the RPC
communication are deleted by calling ``CalculatorServer`` and ``DomainParticipant`` destructors:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorServer.cpp
    :language: cpp
    :start-after: //!--DESTRUCTOR
    :end-before: //!--

The ``main()`` function contains all the steps described above. It runs the ``Server::run()`` method
in a different thread to allow the user to stop the server by sending a signal (for example, Ctrl+C):

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorServer.cpp
    :language: cpp
    :start-after: //!--MAIN
    :end-before: //!--

Client application
^^^^^^^^^^^^^^^^^^

Now, we will create the client application. Running it, a new RPC client will be created
and ready to send requests to the server application. User can send requests
by specifying the operation name and through the CLI, and the results
will be printed on the screen after receiving the reply from the server.

Create a ``CalculatorClient.cpp`` file and copy the following content:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/CalculatorClient.cpp
    :language: cpp

Examining the code
""""""""""""""""""

The ``CalculatorClient.cpp`` file contains the implementation of the ``Client`` class, formed by
a ``Calculator`` client instance and its related ``DomainParticipant``.
Additionally, the operation to be performed is also stored:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--CLIENT_PROTECTED_MEMBERS
    :end-before: //!--

All this members are initialized calling ``init()`` method, in the same way as in the ``Server`` class:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--INIT
    :end-before: //!--

When the client performs an operation (*i.e*: sends a request), three different situations can happen:

* The operation is successful (*i.e* the client sends the request and receives the reply
  from the server).
* The operation fails (*i.e* the client sends the request but an exception occurs). For example, if the
  operation is not implemented in the server side or an RPC exception occurs (for example, if computing
  the result raises an ``OverflowException``).
* A timeout is configured to avoid blocking the thread infinitely if no replies are received for a given
  request and the maximum time is reached.
  In this case, the client will stop waiting for the reply and return a timeout error.

To process each operation in the same way, the following design pattern is used: each operation implements
a ``Operation`` abstract class, which contains a ``execute()`` method:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--OPERATION_INTERFACE
    :end-before: //!--

After calling this method, it will
return an enum ``OperationStatus`` indicating the result of the operation (and addressing each of the
cases previously described):

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--OPERATION_STATUS
    :end-before: //!--

It makes easier to add new operations (for example, ``@feed`` operations) without modifying the
main execution flow. Each operation stores the data required to execute the operation,
for example, a reference to the client used to send the request, as well as the operation input data.

When ``RepresentationLimits`` operation is executed, client sends a request to the server and waits
for the server reply, printing the received result. If something fails or the timeout is exceeded (for example,
if no servers are available), the operation fails:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--REPRESENTATION_LIMITS
    :end-before: //!--

Similarly for ``Addition`` operation:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--ADDITION
    :end-before: //!--

and ``Subtraction`` operation:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--SUBTRACTION
    :end-before: //!--

The operation to be performed is configured from the parsed CLI input
using the ``set_operation()`` factory method. Input operands are hardcoded to simply
the input parsing:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--SET_OPERATION
    :end-before: //!--

When ``send_request()`` method is called, the input operation is configured and the client executes it.
A boolean is returned, ``true`` if the operation is successful or ``false`` otherwise:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--SEND_REQUEST
    :end-before: //!--

Finally, the ``main()`` function process the user input (specifying the operation to be performed),
initializes a new client and tries to execute the operation until a max number of attempts ``n_attempts``:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/code_blocks/CalculatorClient.cpp
    :language: cpp
    :start-after: //!--MAIN
    :end-before: //!--

Note that, before sending the first request, we are waiting some time to make sure
that all internal DDS entities are matched. This way, we avoid losing requests by sending them
before client and server are discovered each other.

Update the *CMakeLists.txt* file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before building the application, we need to update the *CMakeLists.txt* file of the workspace
to add the executable and the required libraries. The following code should be added at the end
of the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/CMakeLists.txt
    :language: cmake
    :lines: 44-56

Build the application
---------------------

Now that all the files are created, the application can be built.
To do so, open a terminal in the *workspace_CalculatorBasic* directory and run the following commands:

.. code-block:: bash

    mkdir build && cd build
    cmake ..
    cmake --build .

The generated executable will be located in the *build* subdirectory.

Run the application
-------------------

To test the application, open two terminals in the *workspace_CalculatorBasic* directory
and execute the following commands:

* In the first terminal, run the server application:

.. code-block:: bash

    ./build/basic_server

* In the second terminal, run the client application and specify the operation to be performed.
  For example, to perform an addition of two numbers (5 and 3), run the following command:

.. code-block:: bash

    ./build/basic_client add

You should see the result of the operation printed on the screen:

.. code-block:: shell-session

    Attempting to send request, attempt 2/10
    Addition result: 5 + 3 = 8
    Request sent successfully

The output of the rest operations should be similar to the following:

* Subtraction:

.. code-block:: shell-session

    ./build/basic_client sub

    Attempting to send request, attempt 2/10
    Subtraction result: 5 - 3 = 2
    Request sent successfully


* Representation limits:

.. code-block:: shell-session

    ./build/basic_client rep

    Attempting to send request, attempt 2/10
    Representation limits received: min_value = -2147483648, max_value = 2147483647
    Request sent successfully

Next steps
----------

The application that we have created only contains basic asynchronous RPC operations.
This example can be extended to include streaming of input and output data by defining
``@feed`` annotated operations in the interface of the IDL file. An example of this can be seen
in the next section (:ref:`fastddsgen_rpc_calculator_feed_app_intro`).
