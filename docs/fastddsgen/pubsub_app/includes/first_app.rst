Generating a minimal functional example
------------------------------------------

If the steps outlined in the Installation Manual have been followed, then *Fast DDS*, *Fast CDR*, and
Fast-RTPS-Gen should be installed in the system.

Generate the Fast DDS source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The application files are generated using the following command.
The ``-example`` option creates an example application, and the CMake files needed to build it.
In the workspace directory (*FastDDSGenHelloWorld* directory), execute one of the following commands according to the
installation followed and the operating system.

* On Linux:

    - For an **installation from binaries** or a **colcon installation**:

    .. code:: bash

        <path-to-Fast-DDS-workspace>/src/fastrtpsgen/scripts/fastrtpsgen -example CMake HelloWorld.idl

    - For a **stand-alone installation**, run:

    .. code:: bash

        <path-to-Fast-DDS-Gen>/scripts/fastrtpsgen -example CMake HelloWorld.idl

* On Windows:

    - For a **colcon installation**:

    .. code:: bash

        <path-to-Fast-DDS-workspace>/src/fastrtpsgen/scripts/fastrtpsgen.bat -example CMake HelloWorld.idl

    - For a **stand-alone installation**, run:

    .. code:: bash

        <path-to-Fast-DDS-Gen>/scripts/fastrtpsgen.bat -example CMake HelloWorld.idl

    - For an **installation from binaries**, run:

    .. code:: bash

        $INST_DIR/src/fastrtpsgen/scripts/fastrtpsgen.bat -example CMake HelloWorld.idl


Build the Fast DDS application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Then, compile the generated code executing the following commands from the *FastDDSGenHelloWorld* directory.

* On Linux:

.. code:: bash

    cd build
    cmake ..
    make

* On Windows:

.. code:: bash

    cd build
    cmake -G "Visual Studio 15 2017 Win64" ..
    cmake --build .

Run the Fast DDS application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The application build can be used to spawn any number of publishers and subscribers associated with the topic.

* On Linux:

.. code:: bash

    ./HelloWorld publisher
    ./HelloWorld subscriber

* On Windows:

.. code:: bash

    HelloWorld.exe publisher
    HelloWorld.exe subscriber


Each time <Enter\> is pressed on the Publisher, a new datagram is generated, sent over the network and receiver by
Subscribers currently online.
If more than one subscriber is available, it can be seen that the message is equally received on all listening nodes.

The values on the custom IDL-generated data type can also be modified as indicated below.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //PUBSUB_API_WRITE_SAMPLE
   :end-before: //!
   :dedent: 4

.. warning::

    It may be necessary to set up a special rule in the Firewall for *eprosima Fast DDS* to work correctly on Windows.
