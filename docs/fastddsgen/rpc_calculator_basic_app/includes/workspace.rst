.. _fastddsgen_rpc_calculator_basic_app_workspace:

Create the application workspace
--------------------------------

First, create a directory named *workspace_CalculatorBasic*, which will represent the workspace of the application.

The workspace will have the following structure at the end of the project.
The file ``build/calculator`` corresponds to the generated *Fast DDS* application:

.. code-block:: shell-session
    
    .
    ├── build
    │   ├── calculator
    │   ├── ...
    ├── CMakeLists.txt
    └── src
        ├── Application.cpp
        ├── Application.hpp
        ├── app_utils.hpp
        ├── ClientApp.cpp
        ├── ClientApp.hpp
        ├── CLIParser.hpp
        ├── main.cpp
        ├── ServerApp.cpp
        ├── ServerApp.hpp
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
