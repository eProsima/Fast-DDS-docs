.. _fastddsgen_rpc_calculator_basic_app_workspace:

Create the application workspace
--------------------------------

First, create a directory named *workspace_CalculatorBasic*, which will represent the workspace of the application.

The workspace will have the following structure at the end of the project.
The files ``build/client`` and ``build/server`` corresponds to
the generated *Fast DDS* client and server applications, respectively:

.. code-block:: shell-session

    .
    ├── build
    │   ├── client
    │   ├── server
    │   ├── ...
    ├── CMakeLists.txt
    └── src
        ├── CalculatorClient.cpp
        ├── CalculatorServer.cpp
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
