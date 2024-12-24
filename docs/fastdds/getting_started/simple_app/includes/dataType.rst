Build the topic data type
^^^^^^^^^^^^^^^^^^^^^^^^^

*eProsima Fast DDS-Gen* is a Java application that generates source code using the data types defined in an
Interface Description Language (IDL) file. This application can do two different things:

1. Generate C++ definitions for your custom topic.
2. Generate a functional example that uses your topic data.

It will be the former that will be followed in this tutorial.
To see an example of application of the latter you can check this other :ref:`example <fastddsgen_pubsub_app>`.
See :ref:`fastddsgen_intro` for further details.
For this project, we will use the Fast DDS-Gen application to define the data type of the messages that will be sent
by the publishers and received by the subscribers.

In the workspace directory, execute the following commands:

.. code-block:: bash

    cd src && touch HelloWorld.idl

This creates the HelloWorld.idl file in the `src` directory.
Open the file in a text editor and copy and paste the following snippet of code.

.. code-block:: omg-idl

    struct HelloWorld
    {
        unsigned long index;
        string message;
    };

By doing this we have defined the ``HelloWorld`` data type, which has two elements: an *index* of type ``uint32_t``
and a *message* of type ``std::string``.
All that remains is to generate the source code that implements this data type in C++11.
To do this, run the following command from the ``src`` directory.

.. code-block:: bash

    <path/to/Fast DDS-Gen>/scripts/fastddsgen HelloWorld.idl

This must have generated the following files:

    * HelloWorld.hpp: HelloWorld type definition.
    * HelloWorldPubSubTypes.cxx: Interface used by Fast DDS to support HelloWorld type.
    * HelloWorldPubSubTypes.h: Header file for HelloWorldPubSubTypes.cxx.
    * HelloWorldCdrAux.ipp: Serialization and Deserialization code for the HelloWorld type.
    * HelloWorldCdrAux.hpp: Header file for HelloWorldCdrAux.ipp.
    * HelloWorldTypeObjectSupport.cxx: |TypeObject| registration code.
    * HelloWorldTypeObjectSupport.hpp: Header file for HelloWorldTypeObjectSupport.cxx.