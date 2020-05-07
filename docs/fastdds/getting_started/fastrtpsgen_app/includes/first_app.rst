Generating a minimal functional example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The FASTRTPSGEN tool is used to build the publication/subscription application.
This is a Java application for building a fully functional publication/subscription application from an Interface
Definition Language (IDL) file that defines the Topic under which messages are published and received.
The application generated allows the creation of as many publishers and subscribers as
desired, all belonging to the same Domain and communicating using the same Topic.

First, locate the installation directories of Fast-RTPS, Fast-CDR and add these directories to your PATH.

    - If you have followed the **colcon installation**, run the following command:

    .. code:: bash

        source <path-to-Fast-RTPS-workspace>/install/setup.bash

    - However, if you have followed the **manual installation** the libraries are already accessible.

The application files are generated using the following command.
The `-example` option creates an example application, and the CMake files needed to build it.

    * On Linux:

        - If you have followed the **colcon installation**:

        .. code:: bash

            <path-to-Fast-RTPS-workspace>/src/fastrtpsgen/scripts/fastrtpsgen -example CMake HelloWorld.idl

        - However, if you have followed the **manual installation**, run:

        .. code:: bash

            <path-to-Fast-RTPS-Gen>/scripts/fastrtpsgen -example CMake HelloWorld.idl

    * On Windows:

        - If you have followed the **colcon installation**:

        .. code:: bash

            <path-to-Fast-RTPS-workspace>/src/fastrtpsgen/scripts/fastrtpsgen.bat -example CMake HelloWorld.idl

        - However, if you have followed the **manual installation**, run:

        .. code:: bash

            <path-to-Fast-RTPS-Gen>/scripts/fastrtpsgen.bat -example CMake HelloWorld.idl

Finally, we compile the generated code.

    * On Linux:

    .. code:: bash

        mkdir build && cd build
        cmake ..
        make

    * On Windows:

    .. code:: bash

        mkdir build && cd build
        cmake -G "Visual Studio 15 2017 Win64" ..
        cmake --build .

The application build can be used to spawn any number of publishers and subscribers associated with your topic.

    On Linux: ::

        ./HelloWorld publisher
        ./HelloWorld subscriber

    On Windows: ::

        HelloWorld.exe publisher
        HelloWorld.exe subscriber


Each time you press <Enter\> on the Publisher, a new datagram is generated, sent over the network and receiver by
Subscribers currently online.
If more than one subscriber is available, it can be seen that the message is equally received on all listening nodes.

You can also modify any values on your custom, IDL-generated data type.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //PUBSUB_API_WRITE_SAMPLE
   :end-before: //!

Take a look at the `eProsima Fast RTPS examples on github <https://github.com/eProsima/Fast-RTPS/tree/master/examples>`_
for ideas on how to improve this basic application through different configuration
options, and for examples of advanced Fast RTPS features.

You may need to set up a special rule in your Firewall for *eprosima Fast RTPS* to work correctly on Windows.
