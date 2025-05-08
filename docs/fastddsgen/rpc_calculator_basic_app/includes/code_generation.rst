.. _fastddsgen_rpc_calculator_basic_app_code_generation:

Generate the Fast DDS source code from the IDL file
---------------------------------------------------

Once the IDL file is created, the application files can be generated using **Fast DDS-Gen**.
In the workspace (*workspace_CalculatorBasic/types* directory), execute one of the following commands according to the
installation followed and the operating system:

* On Linux:

  - For an **installation from binaries** or a **colcon installation**:

  .. code:: bash

      <path-to-Fast-DDS-workspace>/src/fastddsgen/scripts/fastddsgen calculator.idl

  - For a **stand-alone installation**, run:

  .. code:: bash

      <path-to-Fast-DDS-Gen>/scripts/fastddsgen calculator.idl

* On Windows:

  - For a **colcon installation**:

  .. code:: winbatch

      <path-to-Fast-DDS-workspace>/src/fastddsgen/scripts/fastddsgen.bat calculator.idl

  - For a **stand-alone installation**, run:

  .. code:: winbatch

      <path-to-Fast-DDS-Gen>/scripts/fastddsgen.bat calculator.idl

  - For an **installation from binaries**, run:

  .. code:: winbatch

      fastddsgen.bat calculator.idl

.. warning::

    The colcon installation does not build the ``fastddsgen.jar`` file although it does download the Fast DDS-Gen
    repository. The following commands must be executed to build the Java executable:

    .. code-block:: bash

        cd <path-to-Fast-DDS-workspace>/src/fastddsgen
        gradle assemble

After executing the command, the workspace directory will have the following structure:

.. code-block:: shell-session

  .
  ├── CMakeLists.txt
  └── src
      └── types
          ├── calculatorCdrAux.hpp
          ├── calculatorCdrAux.ipp
          ├── calculatorClient.cxx
          ├── calculatorClient.hpp
          ├── calculator_details.hpp
          ├── calculator.hpp
          ├── calculator.idl
          ├── calculatorPubSubTypes.cxx
          ├── calculatorPubSubTypes.hpp
          ├── calculatorServer.cxx
          ├── calculatorServer.hpp
          ├── calculatorServerImpl.hpp
          ├── calculatorTypeObjectSupport.cxx
          └── calculatorTypeObjectSupport.hpp

.. _fastddsgen_rpc_code_generation_basic_example_files_description:

Files description
^^^^^^^^^^^^^^^^^

A description of the generated files is as follows:

calculator
""""""""""

Contains the definition of the interface and its operations:

* ``OverflowException`` class represents the exception defined in the IDL file. It inherits
  from the *eProsima* exception class ``RpcOperationError``, which is
  the base class for all exceptions raised by the *Fast DDS* RPC API when the server communicates an error.

* ``Calculator`` class represents the interface defined in the IDL file. Each operation is defined as
  a pure virtual function, expecting the client to implement it.

Note that, due to the asynchronous nature of *Remote Procedure Calls*, operation calls return a
``RpcFuture`` object, which can be used to retrieve the result of the operation when it is ready.

calculator_details
""""""""""""""""""

According to the
`RPC over DDS specification <https://www.omg.org/spec/DDS-RPC/1.0/PDF>`_ (sections 7.5.1.1.4 and 7.5.1.1.5),
each operation defined in the interface should be mapped to a request type and a reply type,
used in the request/reply topics:

* On one hand, the request type is defined by a ``calculator_<operation_name>_In`` structure,
  containing the *in* and *inout*
  parameters of the operation, in the same order as defined in the IDL file.

* On the other hand, the reply type is defined by ``calculator_<operation_name>_Out``
  and ``calculator_<operator_name>_Result`` structures.
  The first one contains the *out* and *inout* parameters of the operation,
  in the same order as declared in the IDL file.
  The second one contains optional members for the result of the operation and for each esception that
  can be raised.

In the top level, two structures ``Calculator_Request`` and ``Calculator_Reply`` are defined,
which are the types used to publish messages in the request and reply topics.
They contain the previously explained members for each operation.

calculatorClient
""""""""""""""""

Contains the ``CalculatorClient`` class,
which represents a client in the RPC communication and can be instantiated
calling ``create_CalculatorClient`` function.

In a lower level, it makes use of a Requester for sending requests and receiving replies,
which support custom QoS passing it in client creation.

Internally, it implements the pure virtual functions of the
``Calculator`` abstract class, so user can call the operations directly. When a operation method is called,
the client sends a new request using its internal Requester and waits for the reply to be received.

calculatorServer
""""""""""""""""

Contains the ``CalculatorServerLogic`` class, which implements the server for the calculator interface and
can be instantiated by the user calling ``create_CalculatorServer`` function.

``CalculatorServer`` struct represents the public API of the server. User can run or a stop a server calling
``CalculatorServer::run()`` or ``CalculatorServer::stop()`` methods, respectively.

calculatorServerImpl
""""""""""""""""""""

Contains the implementation of the interface methods in the server side.
By default, the server implementation is empty and the user must implement the methods.

For this example, overwrite the methods in the ``calculatorServerImpl.hpp`` file with the following content:

.. literalinclude:: /../code/Examples/C++/RPCClientServerBasic/src/types/calculatorServerImpl.hpp
    :language: cpp


calculatorCdrAux
""""""""""""""""

Contains a set of *Fast CDR* serialization
and deserialization utilities for the types defined in the ``calculator_details.hpp`` file.

calculatorPubSubTypes
"""""""""""""""""""""

Contains the implementation of the methods
required to serialize and deserialize Request and Reply data types.
