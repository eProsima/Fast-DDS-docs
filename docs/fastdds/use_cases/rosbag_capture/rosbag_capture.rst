.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _rosbag_capture:

How to use eProsima DDS Record and Replay (rosbag2 and DDS)
===========================================================

*eProsima DDS Record and Replay* allows the user to continuously monitor the ROS 2 traffic in real time,
and to play it back at any given time.
This highly contributes to facilitating simulation of real life conditions,
application testing, optimizing data analysis and general troubleshooting.
`rosbag2 <https://github.com/ros2/rosbag2>`_ is a ROS 2 application that can be used to capture DDS messages
and store them on an SQLite database which allows inspecting and replaying said messages at a later time.

rosbag2 interactions with a native Fast DDS application
-------------------------------------------------------

Using rosbag2 to capture traffic between ROS 2 talkers and listeners is straightforward.
However, recording and replaying messages sent by Fast DDS participants outside ROS 2 ecosystem requires some
modifications.

Prerequisites
^^^^^^^^^^^^^

A Fast DDS installation, either binary or from sources is required.
Fast DDS-Gen is also required for generating the examples and Fast DDS TypeSupport from the IDL file.
A ROS 2 installation with the rosbag2 package is needed as well.

DDS IDL interoperability with ROS 2 messages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

DDS uses IDLs to define the data model being exchanged by the applications.
While ROS 2 can use IDL files to define the messages, there are some rules that these IDL files must follow so
compatibility between ROS 2 and Fast DDS native applications can be achieved.
Specifically, the type definition must be nested inside the type module name and then the generator to be used.
For ROS 2 messages, the generator would be ``msg``, whereas in this case, the ``idl`` generator must be used.
Assuming that the type module name selected is ``fastdds_record_typesupport`` the following ``HelloWorld.idl``
file could be defined.
This IDL file will be the one used in the following steps.

.. code-block:: omg-idl

   module fastdds_record_typesupport
   {
        module idl
        {
            struct HelloWorld
            {
                unsigned long index;
                string message;
            };
        };
   };

By default, rosbag2 can only recognize those Topics which types ROS 2 has already defined in its different TypeSupport
libraries.
Therefore, a new ROS 2 TypeSupport module library generated with the previously defined types must be created,
so rosbag2 would be able to parse the message contents coming from the Fast DDS application.
First, the new ROS 2 TypeSupport package should be created.
Follow the instructions below, after having sourced your ROS 2 installation:

.. code-block:: bash

   ros2 pkg create --build-type ament_cmake fastdds_record_typesupport

This command will create a new ROS 2 package named ``fastdds_record_typesupport`` with the following folder structure:

.. code-block:: shell-session

    .
    └── fastdds_record_typesupport
        ├── include
        │   └── fastdds_record_typesupport
        ├── src
        ├── CMakeLists.txt
        └── package.xml

ROS 2 TypeSupport code generators expect IDL files inside their own idl folder, so the final folder structure would be
like this:

.. code-block:: shell-session

    .
    └── fastdds_record_typesupport
        ├── idl
        │   └── HelloWorld.idl
        ├── include
        │   └── fastdds_record_typesupport
        ├── src
        ├── CMakeLists.txt
        └── package.xml

In order to generate the TypeSupport interfaces required, the CMakeLists.txt file should be modified accordingly so the
ROS 2 TypeSupport generator is called.
Please add the following lines to the CMakeLists.txt file before calling ``ament_package()``:

.. code-block:: cmake

   find_package(rosidl_default_generators REQUIRED)

   set(idl_files
     "idl/HelloWorld.idl"
   )

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${idl_files}
   )

Similarly, the ``package.xml`` file should be modified adding the ROS 2 TypeSupport generator dependency.
Add the following lines to the ``package.xml`` file after the ``buildtool_depend`` tags:

.. code-block:: bash

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

The last step would be to build the package.
Run the following command within the ``fastdds_record_typesupport`` folder:

.. code-block:: bash

   RMW_IMPLEMENTATION=rmw_fastrtps_cpp colcon build

The build process will create inside the install folder a new ROS 2 overlay with all the required
libraries and scripts for the ROS 2 applications to use te type defined in the IDL file.

Fast DDS Application tuning
^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 adds special tokens to the topic names depending on the ROS 2 subsystem the topic belongs to.
More information on this topic can be found `on ROS 2 design documentation
<https://design.ros2.org/articles/topic_and_service_names.html#examples-of-ros-names-to-dds-concepts>`_ .

Using the same IDL file defined earlier, Fast DDS-Gen can generate the required code to handle the new type in
Fast DDS.
The changes required in the Fast DDS application so rosbag2 can communicate with it are going to be illustrated via the
Publisher/Subscriber example generated automatically from an IDL using Fast DDS-Gen.
An in-depth guide to Fast DDS-Gen can be found
`here <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/fastddsgen/fastddsgen.html>`_.

In the case of plain topics, the namespace "rt/" is added by ROS 2 to the DDS topic name.
DataType names for ROS 2 generated types are structured concatenating the modules names.
For the IDL being used in this example the data type name would be "fastdds_record_typesupport::idl::HelloWorld".

Create a new workspace different from the ROS 2 one used previously.
Copy inside the same IDL file and run Fast DDS-Gen to generate
the TypeSupport and the example source files:

.. code-block:: bash

    mkdir HelloWorldExample
    cd HelloWorldExample
    cp <PATH_TO_ROS2_WORKSPACE>/fastdds_record_typesupport/idl/HelloWorld.idl .
    fastddsgen -example CMake -typeros2 HelloWorld.idl

This command will populate the current folder with the required header and source files to build the TypeSupport,
and the Publisher and Subscriber applications.

.. code-block:: shell-session

    └── HelloWorldExample
        ├── CMakeLists.txt
        ├── HelloWorldCdrAux.hpp
        ├── HelloWorldCdrAux.ipp
        ├── HelloWorld.hpp
        ├── HelloWorld.idl
        ├── HelloWorldPublisher.cxx
        ├── HelloWorldPublisher.h
        ├── HelloWorldPubSubMain.cxx
        ├── HelloWorldPubSubTypes.cxx
        ├── HelloWorldPubSubTypes.h
        ├── HelloWorldSubscriber.cxx
        └── HelloWorldSubscriber.h

The Fast DDS-Gen example should be modified taking into account the topic and type name mangling
applied by ROS 2 so communication can be established with rosbag2.
Having used the ``-typeros2`` Fast DDS-Gen option when generating the TypeSupport, the generated type
name would already include the ROS 2 naming rule mangling.
However, the topic name must be modified manually both in the Publisher and Subscriber applications.
Look for the ``create_topic`` command in both the ``HelloWorldPublisher.cxx`` and the ``HelloWorldSubscriber.cxx``
files and modify the topic name:

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //CREATE THE TOPIC FOR ROSBAG
    :end-before: //!
    :dedent: 4

To build this example run the following commands:

.. code-block:: bash

    mkdir build && cd build
    cmake ..
    make

This will create a HelloWorld binary file inside the build directory that can be used to launch both the Publisher
and the Subscriber applications.
Run each application in a terminal and confirm that the communication is established.

.. code-block:: bash

    ./HelloWorld publisher|subscriber


eProsima DDS Record and Replay
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to use the generated ROS 2 TypeSupport package, the ROS 2 workspace should be sourced besides the
ROS 2 installation.
This allows rosbag2 to record the data types used in this example.
To start recording the traffic being exchanged between the Publisher/Subscriber applications the corresponding
ROS 2 Topic name has to be passed to rosbag2 (not to be mistaken with the DDS Topic name).
Remember also to ensure that Fast DDS is the ROS 2 middleware being used by setting the environment variable
``RMW_IMPLEMENTATION``.

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   source <PATH_TO_ROS2_WORKSPACE>/fastdds_record_typesupport/install/setup.bash
   ros2 bag record /HelloWorldTopic

Having the Publisher application running already, the following rosbag2 log discovery info would be shown:

.. code-block:: bash

   [INFO] [1644320308.422161532] [rosbag2_recorder]: Subscribed to topic '/HelloWorldTopic'
   [INFO] [1644320308.422292205] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...

rosbag2 will proceed to create a folder named ``rosbag2_<DATE>`` with an SQLite database inside (``db3`` extension)
where the received messages will be recorded.
Within the folder a ``YAML`` file provides metadata information about the record: type and topic name, number of
messages recorded, record duration, etc.
The path to this database file can be used to replay the recorded messages.
Having the Subscriber application running, the previously recorded traffic will be replayed.
After stopping the rosbag2 application, rerun it in replay mode running the following command.
The recorded messages will be published by rosbag2 at their original publishing rate and the Subscriber application
will receive them:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   source <PATH_TO_ROS2_WORKSPACE>/fastdds_record_typesupport/install/setup.bash
   ros2 bag play <path-to-db-file>
