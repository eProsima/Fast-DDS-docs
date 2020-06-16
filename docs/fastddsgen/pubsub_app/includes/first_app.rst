Generating a minimal functional example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you have followed the steps outlined in the Installation Manual, you should have *Fast DDS*, Fast CDR, and
Fast-RTPS-Gen installed.

Generate the Fast DDS source code
***********************************

The application files are generated using the following command.
The `-example` option creates an example application, and the CMake files needed to build it.

    * On Linux:

        - If you have followed the **colcon installation**:

        .. code:: bash

            <path-to-Fast-DDS-workspace>/src/fastrtpsgen/scripts/fastrtpsgen -example CMake HelloWorld.idl

        - However, if you have followed the **stand-alone installation**, run:

        .. code:: bash

            <path-to-Fast-DDS-Gen>/scripts/fastrtpsgen -example CMake HelloWorld.idl

    * On Windows:

        - If you have followed the **colcon installation**:

        .. code:: bash

            <path-to-Fast-DDS-workspace>/src/fastrtpsgen/scripts/fastrtpsgen.bat -example CMake HelloWorld.idl

        - However, if you have followed the **stand-alone installation**, run:

        .. code:: bash

            <path-to-Fast-DDS-Gen>/scripts/fastrtpsgen.bat -example CMake HelloWorld.idl

Build the Fast DDS application
********************************

Then, compile the generated code.

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

Run the Fast DDS application
*****************************

The application build can be used to spawn any number of publishers and subscribers associated with your topic.

    * On Linux:

    .. code:: bash

        ./HelloWorld publisher
        ./HelloWorld subscriber

    * On Windows:

    .. code:: bash

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
   :dedent: 4

.. warning::

    You may need to set up a special rule in your Firewall for *eprosima Fast DDS* to work correctly on Windows.
