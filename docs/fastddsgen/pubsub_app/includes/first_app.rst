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

.. tab-set::

    .. tab-item:: Linux
        :sync: linux

        - For an **installation from binaries** or a **colcon installation**:

        .. code:: bash

            <path-to-Fast-DDS-workspace>/src/fastddsgen/scripts/fastddsgen -example CMake HelloWorld.idl

        - For a **stand-alone installation**, run:

        .. code:: bash

            <path-to-Fast-DDS-Gen>/scripts/fastddsgen -example CMake HelloWorld.idl

    .. tab-item:: Windows
        :sync: windows

        - For a **colcon installation**:

        .. code:: winbatch

            <path-to-Fast-DDS-workspace>/src/fastddsgen/scripts/fastddsgen.bat -example CMake HelloWorld.idl

        - For a **stand-alone installation**, run:

        .. code:: winbatch

            <path-to-Fast-DDS-Gen>/scripts/fastddsgen.bat -example CMake HelloWorld.idl

        - For an **installation from binaries**, run:

        .. code:: winbatch

            fastddsgen.bat -example CMake HelloWorld.idl

.. warning::

    The colcon installation does not build the ``fastddsgen.jar`` file although it does download the Fast DDS-Gen
    repository. The following commands must be executed to build the Java executable:

    .. code-block:: bash

        cd <path-to-Fast-DDS-workspace>/src/fastddsgen
        gradle assemble


Build the Fast DDS application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Then, compile the generated code executing the following commands from the *FastDDSGenHelloWorld* directory.

.. tab-set::

    .. tab-item:: Linux
        :sync: linux

        .. code:: bash

            cd build
            cmake ..
            make

    .. tab-item:: Windows
        :sync: windows

        .. code-block:: bash

            cd build
            cmake -G "Visual Studio 16 2019" -A x64 ..
            cmake --build .


Run the Fast DDS application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The application build can be used to spawn any number of publishers and subscribers associated with the topic.

.. tab-set::

    .. tab-item:: Linux
        :sync: linux

        .. code:: bash

            ./HelloWorld publisher
            ./HelloWorld subscriber

    .. tab-item:: Windows
        :sync: windows

        .. code-block:: bash

            HelloWorld.exe publisher
            HelloWorld.exe subscriber


Each time <Enter\> is pressed on the Publisher, a new datagram is generated, sent over the network and received by
Subscribers currently online.
If more than one subscriber is available, it can be seen that the message is equally received on all listening nodes.

The values on the custom IDL-generated data type can also be modified as indicated below.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //PUBSUB_API_WRITE_SAMPLE
   :end-before: //!
   :dedent: 4

.. warning::

    It may be necessary to set up a special rule in the Firewall for *eprosima Fast DDS* to work correctly on Windows.
