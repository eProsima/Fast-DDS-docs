Build the client/server interface
---------------------------------

The operations of the calculator service should be specified in an IDL file using an interface and
following the syntax explained in :ref:`fastddsgen_interfaces_definition`.

In the workspace directory, run the following commands:

.. code-block:: bash

    mkdir src && cd src
    mkdir types && cd types
    touch calculator.idl
    cd ../..

We have created a separated ``types`` subdirectory inside the workspace source directory to separate the source code
generated by *Fast DDS-Gen* from the rest of the application code. Now open the *calculator.idl* file
with a text editor and copy the following content inside it:

.. literalinclude:: /../code/Examples/C++/RpcClientServerBasic/src/types/calculator.idl
    :language: omg-idl

For additions and subtractions, an overflow exception is raised in case of working with
operands which produce a result that cannot be represented in a 32-bit integer.
For more information about how to define exceptions in IDL files, please check :ref:`fastddsgen_interfaces_exceptions`.
