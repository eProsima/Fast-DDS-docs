Generate the source code for the application
--------------------------------------------

Now that the interface source code has been generated, the next step is to generate the application source code.
All the following files should be created in the ``workspace_CalculatorBasic/src`` directory.

CLI and logging related files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following files are required to implement the CLI and logging functionalities of the application, and they are not
directly related to the RPC functionality.

First, create *Application.hpp* and *Application.cpp* files, which contains the abstract Application class and a factory
method to create both client and server applications, respectively. Copy the following code into the files:

* Application.hpp


..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/Application.hpp
    :language: cpp

* Application.cpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/Application.cpp
    :language: cpp


Now, create a *CLIParser.hpp* file, which contains the ``CLIParser`` class that implements the command line parser.
Copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/CLIParser.hpp
    :language: cpp

After that, create a *app_utils.hpp* file, which contains some logging utilities for our applications.
Copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/app_utils.hpp
    :language: cpp

Finally, create a *main.cpp* file, which contains the main function of the application, and allows the user to
close it by sending a signal. Copy the following code into the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/main.cpp
    :language: cpp

.. _fastddsgen_rpc_calculator_basic_server_application:

Server application
^^^^^^^^^^^^^^^^^^

The next step is to create the server application. Running it, the user can run an RPC server
ready to process requests from client applications and send replies.

First, create both ServerApp.hpp and ServerApp.cpp files, which contains the ``ServerApp`` class
that represents the server application. Copy the following code into the files:

* ServerApp.hpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/ServerApp.hpp
    :language: cpp

* ServerApp.cpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/ServerApp.cpp
    :language: cpp

Examining the code
""""""""""""""""""

ServerApp.hpp
*************

The *ServerApp.hpp* contains the declaration of the ``ServerApp`` class, which represents
the server application. It is intended to implemement the abstract ``Application`` interface:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.hpp
    :language: cpp
    :start-after: //!--CLASS_NAME
    :end-before: //!--

Thus, it overrides the ``Application::run()`` and ``Application::stop()`` methods,
which are responsible for running and stopping the server, respectively.

Additionally, a ``ServerApp`` instance is created passing the configuration
parsed thought the CLI, and the name of the RPC service in which the server is
involved:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.hpp
    :language: cpp
    :start-after: //!--PUBLIC_METHODS
    :end-before: //!--

Finally, ``ServerApp`` class will implement some private methods to handle the
server setup, including the creation of a DDS DomainParticipant and the ``CalculatorServer``
instance. In addition, a ``ServerApp::is_stopped()`` method is implemented to check
if the server execution will finish (for example, after receiving a signal).

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.hpp
    :language: cpp
    :start-after: //!--PRIVATE_METHODS
    :end-before: //!--

``DomainParticipant`` and ``CalculatorServer`` instances, as well as the application state and
each configurable parameter are stored as private members:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.hpp
    :language: cpp
    :start-after: //!--PRIVATE_MEMBERS
    :end-before: //!--

ServerApp.cpp
*************

The *ServerApp.cpp* file contains the implementation of the ``ServerApp`` class.
When the server application is started, server related CLI psrameters are stored and
new ``DomainParticipant`` and ``CalculatorServer`` instances are created:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.cpp
    :language: cpp
    :start-after: //!--CONSTRUCTOR
    :end-before: //!--

On one hand, we will use a ``DomainPartipant`` using the default QoS settings:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.cpp
    :language: cpp
    :start-after: //!--CREATE_PARTICIPANT
    :end-before: //!--

On the other hand, the ``CalculatorServer`` instance is created using the previously created
participant, the server parameters parsed through the CLI, the name of te RPC service and
the server-side implementation of the methods defined in the IDL file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.cpp
    :language: cpp
    :start-after: //!--CREATE_SERVER
    :end-before: //!--

When the server application starts running, the RPC server is started using the
``CalculatorServer::run()`` method. Similarly, when the application is stopped, the
RPC server is also stopped using the ``CalculatorServer::stop()`` method:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.cpp
    :language: cpp
    :start-after: //!--RUN/STOP
    :end-before: //!--

Before finishing the application, all the internal DDS and RPC entities involved in the RPC
communication are deleted by calling ``CalculatorServer`` and ``DomainParticipant`` destructors:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ServerApp.cpp
    :language: cpp
    :start-after: //!--DESTRUCTOR
    :end-before: //!--

Client application
^^^^^^^^^^^^^^^^^^

Now, we will create the client application. Running it, a new RPC client will be created
and ready to send requests to the server application. User can send requests
by specifying the operation name and the input data through the CLI, and the results
will be printed on the screen after receiving the reply from the server.

First, create both *ClientApp.hpp* and *ClientApp.cpp* files, which contains the ``ClientApp`` class
that represents the client application. Copy the following code into the files:

* ClientApp.hpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/ClientApp.hpp
    :language: cpp

* ClientApp.cpp

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/clean_files/ClientApp.cpp
    :language: cpp

Examining the code
""""""""""""""""""

ClientApp.hpp
*************

The *ClientApp.hpp* file contains the declaration of the ``ClientApp`` class, which represents
the client application. It is intended to implement the abstract ``Application`` interface:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--CLASS_NAME
    :end-before: //!--

Thus, it overrides the ``Application::run()`` and ``Application::stop()`` methods,
which are responsible for running and stopping the client, respectively.
Additionally, a ``ClientApp`` instance is created passing the configuration
parsed thought the CLI, and the name of the RPC service in which the client is
involved:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--PUBLIC_METHODS
    :end-before: //!--

Finally, ``ClientApp`` class will implement some private methods to handle the
client setup. A description of each method is provided below:

* ``ClientApp::create_participant()``: creates a DDS DomainParticipant using the default QoS settings.
* ``ClientApp::create_client()``: creates a RPC client using the previously created participant,
  and the name of the RPC service.
* ``ClientApp::set_operation()``: sets the operation to be performed by the client based on the operation
  specified through the CLI. Additionally, a Ping operation can also be configured to check if at least
  one server is available and avoid blocking infinitely the thread when sending a request.
* ``ClientApp::is_stopped()``: checks if the client execution will finish (for example, after receiving a signal).

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--PRIVATE_METHODS
    :end-before: //!--

``DomainParticipant`` and ``Calculator`` client instances, as well as the application state, the operation
to be performed and each configurable parameter, are stored as private members:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--PRIVATE_METHODS
    :end-before: //!--

When the client performs an operation (*i.e*: sends a request), three different situations can happen:

* The operation is performed succesfully (*i.e* the client sends the request and receives the reply
  from the server).
* The operation fails (*i.e* the client sends the request but an exception occurs). For example, if the
  operation is not implemented in the server side or an RPC exception occurs (for example, if computing
  the result raises an ``OverflowException``).
* A timeout is configured to avoid blocking the thread infinitely if no replies are received for a given
  request and the maximum time is reached (in our case, when the client tests the connectivity with the server).
  In this case, the client will stop waiting for the reply and return a timeout error.

To process each operation in the same way, the following design pattern is used: each operation implements
a ``Operation`` abstract class, which contains a ``execute()`` method. After calling this method, it will
return an enum ``OperationStatus`` indicating the result of the operation (and addressing each of the
cases previously described):

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--OPERATION_INTERFACE
    :end-before: //!--

It makes easier to add new operations (for example, ``@feed`` operations) without modifying the
``ClientApp::run()`` execution flow. Each operation stores the data required to perfom the operation,
for example, a reference to the client used to send the request, as well as the input data provided
by the user thorugh the CLI and the result of the operation:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.hpp
    :language: cpp
    :start-after: //!--OPERATIONS
    :end-before: //!--

A more detailed description of each operation execution is provided in the following subsection.

ClientApp.cpp
*************

The *ClientApp.cpp* file contains the implementation of the ``ClientApp`` class.
When the client application is started, client related CLI parameters are stored and
new ``DomainParticipant`` and ``Calculator`` client instances are created:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--CONSTRUCTOR
    :end-before: //!--

Similarly to the server application, a ``DomainPartipant`` using the default QoS settings is created:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--CREATE_PARTICIPANT
    :end-before: //!--

and the ``Calculator`` client is created using the previously created participant, with the default
Requester QoS settings:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--CREATE_CLIENT
    :end-before: //!--

On the other hand, the operation to be performed is internally configured by checking the configuration
provided by the user, or configured to perform a Ping operation in case of testing the connectivity
with the server:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--SET_OPERATION
    :end-before: //!--

When the client application starts running, the following steps are performed:

#. The connectivity with the server is tested by sending a Ping operation. If the server is not available,
   the client execution will finish and a runtime exception will be thrown. The number of connection
   attempts can be specified by the user using the ``--connection-attempts`` parameter in CLI.
   Before retrying, the client will wait for a given time (in this case, 1 second).

#. If no signals to stop the application are received, the user-specified operation is configured and performed.
   If something goes wrong, and error message will be printed on the screen and the application will finish throwing
   a runtime exception.

#. The application is stopped by calling the ``ClientApp::stop()`` method, which will stop the client and
   delete all the internal DDS and RPC entities involved in the RPC communication.

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--RUN
    :end-before: //!--

The ping operation can be any of the implemented operations in the server side
(for example, an arbitrary addition). After sending the request, client waits some time for
the reply. If the reply is not received, the operation will return a ``OperationStatus::TIMEOUT``
code. If the reply is received, at least one server is available so the operation returns a
``OperationStatus::SUCCESS`` code. If some ``RpcException`` occurs, an error will be printed and
the operation will return a ``OperationStatus::ERROR`` code:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--PING
    :end-before: //!--

The implementation of the rest of operations is similar. For ``RepresentationLimits``, ``min_value_``
and ``max_value_`` represents the *out* parameters of the operations, specified in the IDL interface.
They are passed by reference to the client implementation using the ``client_->representation_limits()`` call,
and will be filled when the result of the request is received (*i.e*: after ``future.get()`` execution):

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--REPRESENTATION_LIMITS
    :end-before: //!--

For ``Addition`` and ``Substraction`` operations, the user input data is stored in ``x_`` and ``y_``
members, and passed to the client implementation as input parameters in ``client_->addition()`` call.
When ready, the result is stored in ``result_`` member and printed on the screen:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--ADDITION
    :end-before: //!--

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/ClientApp.cpp
    :language: cpp
    :start-after: //!--SUBSTRACTION
    :end-before: //!--

Update the *CMakeLists.txt* file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before building the application, we need to update the *CMakeLists.txt* file of the workspace
to add the executable and the required libraries. The following code should be added at the end
of the file:

..  literalinclude:: /../code/Examples/C++/RPCClientServerBasic/CMakeLists.txt
    :language: cmake
    :lines: 45-50

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

To test the application, open two terminals in the *workspace_CalculatorBasic/build* directory
and execute the following commands:

* In the first terminal, run the server application:

.. code-block:: bash

    ./build/calculator server

* In the second terminal, run the client application and specify the operation to be performed.
  For example, to perform an addition of two numbers, run the following command:

.. code-block:: bash

    ./build/calculator client --addition 5 3

You should see the result of the operation printed on the screen:

.. code-block:: shell-session

    2025-05-08T15:07:29.827 [INFO] [ClientApp] Addition result: 5 + 3 = 8
    2025-05-08T15:07:29.827 [INFO] [ClientApp] Operation finished. Stopping client execution...
    2025-05-08T15:07:29.827 [INFO] [ClientApp] Client execution stopped

You can check all the available CLI options showing the help:

.. code-block:: bash

    ./build/calculator -h

The output of the rest operations should be similar to the following:

* Subtraction:

.. code-block:: shell-session

    ./build/calculator client -s 5 3

    2025-05-08T15:08:25.873 [INFO] [ClientApp] Server reachable
    2025-05-08T15:08:25.874 [INFO] [ClientApp] Subtraction result: 5 - 3 = 2
    2025-05-08T15:08:25.874 [INFO] [ClientApp] Operation finished. Stopping client execution...
    2025-05-08T15:08:25.874 [INFO] [ClientApp] Client execution stopped

* Representation limits:

.. code-block:: shell-session

    ./build/calculator client -r

    2025-05-08T15:09:21.647 [INFO] [ClientApp] Server reachable
    2025-05-08T15:09:21.648 [INFO] [ClientApp] Representation limits received: min_value = -2147483648, max_value = 2147483647
    2025-05-08T15:09:21.648 [INFO] [ClientApp] Operation finished. Stopping client execution...
    2025-05-08T15:09:21.648 [INFO] [ClientApp] Client execution stopped

Next steps
----------

The application that we have created only contains basic asynchronous RPC operations.
This example can be extended to include streaming of input and output data by defining
``@feed`` annotated operations in the interface of the IDL file. An example of this can be seen
in the next section (:ref:`fastddsgen_rpc_calculator_feed_app_intro`).
