Build the topic data type
"""""""""""""""""""""""""

Briefly, *eProsima FASTRTPSGEN* is a Java application that generates source code using the data types defined in an
IDL file.
See :ref:`fastrtpsgen_intro` for further details.

For this project, we will use the FASTRTPSGEN application to define the data type of the messages that will be sent
by the publishers and received by the subscribers.

In the workspace directory, execute the following commands:

.. code-block:: bash

    cd src && touch HelloWorld.idl

This creates the HelloWorld.idl file in the `src` directory.
Open the file in your favorite text editor and copy and paste the following snippet of code.

.. code-block:: idl

    struct HelloWorld
    {
        unsigned long index;
        string message;
    };

With this we have defined the ``HelloWorld`` data type.
This has two elements, an *index* of type ``uint32_t`` and a *message* of type ``std::string``.

All that remains is to generate the source code that implements this data type in C++11. To do this, run the following
command from the ``src`` directory.

.. code-block:: bash

    <path/to/Fast-RTPS-Gen>/scripts/fastrtpsgen HelloWorld.idl

This must have generated the following files:

    * HelloWorld.cxx: HelloWorld type definition.
    * HelloWorld.h: Header file for HelloWorld.cxx.
    * HelloWorldPubSubTypes.cxx: Serialization and Deserialization code for the HelloWorld type.
    * HelloWorldPubSubTypes.h: Header file for HelloWorldPubSubTypes.cxx.

CMakeLists.txt
**************

Include at the end of the CMakeList.txt file you created earlier the following code snippet.
This includes the files we have just created.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/CMakeLists.txt
    :language: bash
    :lines: 44-45
