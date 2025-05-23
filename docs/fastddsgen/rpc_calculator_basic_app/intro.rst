.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fastddsgen_rpc_calculator_basic_app_intro:

Building a RPC Client/Server application
========================================

*Fast DDS-Gen* can be used to generate the source code required to
build a *Remote Procedure Calls* (RPC) Client/Server application from an IDL file.

This section provides an overview of the steps required to build a *Fast DDS* RPC application from scratch,
following the `RPC over DDS specification <https://www.omg.org/spec/DDS-RPC/1.0/PDF>`_, using the *Fast DDS-Gen* tool.

The example consists of a simple CLI tool for a calculator service that allows the client
to call asynchronously the following operations:

* **addition**: Adds two 32-bit integers and returns the result.
* **subtraction**: Subtracts two 32-bit integers and returns the result.
* **representation_limits**: Returns the minimum and maximum representable values for a 32-bit integer.

The entire example source code is available in this
`link <https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerBasic>`_

.. include:: includes/background.rst
.. include:: includes/prerequisites.rst
.. include:: includes/workspace.rst
.. include:: includes/dependencies.rst
.. include:: includes/cmake.rst
.. include:: includes/idl.rst
.. include:: includes/code_generation.rst
.. include:: includes/app.rst
