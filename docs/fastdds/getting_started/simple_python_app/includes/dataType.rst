.. _writing_pubsub_python_datatype:

Build the topic data type
^^^^^^^^^^^^^^^^^^^^^^^^^

*eProsima Fast DDS-Gen* is a Java application that generates source code using the data types defined in an
Interface Description Language (IDL) file.
This application can do two different things:

1. Generate C++ definitions for your custom topic.
2. Generate `SWIG`_ interface files to generate the Python bindings for your custom topic.

For this project, we will use the Fast DDS-Gen application to define the data type of the messages that will be sent
by the publishers and received by the subscribers.

In the workspace directory, execute the following commands:

.. code-block:: bash

    touch HelloWorld.idl

This creates the HelloWorld.idl file.
Open the file in a text editor and copy and paste the following snippet of code.

.. code-block:: omg-idl

    struct HelloWorld
    {
        unsigned long index;
        string message;
    };

By doing this we have defined the ``HelloWorld`` data type, which has two elements: an *index* of type ``uint32_t``
and a *message* of type ``std::string``.
All that remains is to generate the source code that implements this data type in C++11 and the
`SWIG`_ interface files for the Python bindings.
To do this, run the following command.

.. code-block:: bash

    <path/to/Fast DDS-Gen>/scripts/fastddsgen -python HelloWorld.idl

This must have generated the following files:

    * HelloWorld.cxx: HelloWorld C++ type definition.
    * HelloWorld.h: C++ header file for HelloWorld.cxx.
    * HelloWorld.i: `SWIG`_ interface file for HelloWorld C++ type definition.
    * HelloWorldPubSubTypes.cxx: C++ interface used by Fast DDS to support HelloWorld type.
    * HelloWorldPubSubTypes.h: C++ header file for HelloWorldPubSubTypes.cxx.
    * HelloWorldPubSubTypes.i: `SWIG`_ interface file for C++ Serialization and Deserialization code.
    * HelloWorldCdrAux.ipp: C++ serialization and deserialization code for the HelloWorld type.
    * HelloWorldCdrAux.hpp: C++ header file for HelloWorldCdrAux.ipp.
    * CMakeLists.txt: CMake file to generate C++ source code and Python module from the `SWIG`_ interface files,
      compile and generate C++ libraries.

CMakeLists.txt
""""""""""""""
At this point the project is ready for building, compiling and generating Python bindings for this data type.
From the workspace, run the following commands.

.. code-block:: bash

    cmake .
    make

.. _SWIG: http://www.swig.org/
