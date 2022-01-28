.. _writing_pubsub_python_datatype:

Build the topic data type
^^^^^^^^^^^^^^^^^^^^^^^^^^

*eProsima Fast DDS-Gen* is a Java application that generates source code using the data types defined in an
Interface Description Language (IDL) file.
This application can do two different things:

1. Generate C++ definitions for your custom topic.
2. Generate `SWIG`_ inteface files to generate the Python bindings for your custom topic.

For this project, we will use the Fast DDS-Gen application to define the data type of the messages that will be sent
by the publishers and received by the subscribers.

In the workspace directory, execute the following commands:

.. code-block:: bash

    touch HelloWorld.idl

This creates the HelloWorld.idl file.
Open the file in your favorite text editor and copy and paste the following snippet of code.

.. code-block:: idl

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
    * HelloWorldPubSubTypes.cxx: C+`Serialization and Deserialization code for the HelloWorld type.
    * HelloWorldPubSubTypes.h: C++ header file for HelloWorldPubSubTypes.cxx.
    * HelloWorldPubSubTypes.i: `SWIG`_ interface file for C++ Serialization and Deserialization code.
    * CMakeLists.txt: CMake file to generate C++ source code and Python module from the `SWIG`_ interface files,
      compile and generate C++ libraries.
    * HelloWorld.py: Python module to be imported by your Python example.

CMakeLists.txt
""""""""""""""
At this point you can build, compile and generate Python bindings for your data types.
From the workspace, run the following commands.

.. code-block:: bash

    cmake .
    make

.. _SWIG: http://www.swig.org/
