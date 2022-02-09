.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _rosbag_capture:

How to use eProsima DDS Record and Replay (ROSBAG2 and DDS)
===========================================================

.. warning::
    This section is still under work.

eProsima DDS Record and Replay allows the user to monitor the ROS 2 traffic in real time continuously,
and to play it back at any given time. This highly contributes to facilitating simulation of real life
conditions, application testing, optimizing data analysis and general troubleshooting.
Rosbag2 is a ROS 2 application that can be used to capture DDS messages and store them on an sqlite database.
This allows inspecting and replaying said messages at a later time.

Rosbag2 interactions with a native Fast DDS application
-------------------------------------------------------

Using rosbag2 to capture traffic between ROS talkers and listeners is straightforward.
Using it to record and replay messages sent by Fast DDS participants outside ROS2 ecosystem requires some
modifications.

Prerequisites
^^^^^^^^^^^^^
Being a ROS 2 application, to install rosbag requires to have the ROS 2 repository for the ROS 2 distribution of
choice set up in your sources file. Instructions on how to do so can be found on the `ROS 2 documentation
<https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html#add-the-ros-2-apt-repository>`_

Rosbag2 can then be installed following the instructions on their `Github repository
<https://github.com/ros2/rosbag2>`_

IDL files
^^^^^^^^^
IDL files used to generate types for the application need to have the types nested inside both the module name
(new_typesupport in our example) and the "idl" module (which defines the kind of generator to use. Other syntaxes
such as ROS 2 messages would use "msg") in that order. As an example, thee could be the contents of an example
HelloWorld.idl file that we will be using for the next steps of this guide:

.. code-block:: idl

   module new_typesupport{
     module idl {
         struct HelloWorld
           {
               unsigned long index;
               string message;
           };
     };
   };


TypeSupport Generation
^^^^^^^^^^^^^^^^^^^^^^
.. warning::
    This section is still under work.

Rosbag by default can only recognize Topics of the types ROS has already defined in its different TypeSupport
libraries. We need to create a new library containing our custom types and have rosbag find it and use it to
parse our message contents. To do so, start by creating a new package. Assuming that the ROS distribution
that was downloaded in a previous step is ROS 2 Galactic the commands to create a new package would be:

.. code-block:: bash

   source /opt/ros/galactic/setup.bash
   ros2 pkg create --build-type ament_cmake new_typesupport

This will create a new_typesupport folder. This will create a folder structure similar to the following:

.. code-block:: shell-session

    .
    └── new_typesupport
        ├── include
        │   └── new_typesupport
        ├── src
        ├── CMakeLists.txt
        └── package.xml

ROS2 code generators expect IDL files inside their own idl folder, so the final folder structure would be
like this:

.. code-block:: shell-session

    .
    └── new_typesupport
        ├── idl
        │   └── HelloWorld.idl
        ├── include
        │   └── new_typesupport
        ├── src
        ├── CMakeLists.txt
        └── package.xml

Now add the following to your CMakeLists.txt file just before the ament_package() statement so ROS's
generators get called on your IDL files.

.. code-block:: cmake

   find_package(rosidl_default_generators REQUIRED)

   set(idl_files
     "idl/HelloWorld.idl"
   )

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${idl_files}
   )

Similarly, we need to add the ROS generators to the package.xml file so colcon can keep track of this
dependency. Add the following to the package.xml file after the ament_cmake build_depend tags:

.. code-block:: bash

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

The last step would be to build the package. From the new_typesupport folder, the command to run is:

.. code-block:: bash

   colcon build

The build process will create inside the install folder a new ROS2 overlay with all the required libraries
and scripts for ROS2 applications to use your new type.

Fast DDS Application required modifications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This section is still under work.

Using the same IDL files used earlier Fast-DDS-Gen can generate the required code to handle the new type in
Fast DDS. The required changes to make so rosbag can see your application are going to be illustrated via the
Publisher/Subscriber example generated automatically from an IDL using Fast DDS Gen:

Create a new folder outside the new_typesupport folder created previously, go inside it and run fastdds to generate
the types and examples:

.. code-block:: bash

    mkdir HelloWorldExample
    cd HelloWorldExample
    cp ../new_typesupport/idl/HelloWorld.idl .
    fastddsgen -example CMake -typeros2 HelloWorld.idl

This will populate the current folder with several header and source files for both a Publisher and a Subscriber
application.

.. code-block:: shell-session

    └── HelloWorldExample
        ├── CMakeLists.txt
        ├── HelloWorld.cxx
        ├── HelloWorld.h
        ├── HelloWorld.idl
        ├── HelloWorldPublisher.cxx
        ├── HelloWorldPublisher.h
        ├── HelloWorldPubSubMain.cxx
        ├── HelloWorldPubSubTypes.cxx
        ├── HelloWorldPubSubTypes.h
        ├── HelloWorldSubscriber.cxx
        └── HelloWorldSubscriber.h


ROS2 adds special tokens to the topic names depending on the vendor and the kind of topic.
In this case the token "rt/" is added to all topic names so it needs to be added to the topic name.
DataType names for this generated types are structured like (using the previous IDL as example)
"new_typesupport::idl::HelloWorld". It's easier however to use the accessor provided to this value during
the create_topic call. Both of these modifications can be seen here (Line 86 on the HelloWorldPublisher.cxx
generated file):

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //CREATE THE TOPIC FOR ROSBAG
    :end-before: //!
    :dedent: 4

Rosbag Record and Play
^^^^^^^^^^^^^^^^^^^^^^
Using the overlay created earlier one can make ROS applications aware of our custom type. Sourcing the type
support package and then launching rosbag2 will allow our messages to be captured. The topic name used in the
rosbag calls should not have the /rt token discussed earlier. It has to, however, keep the leading forward slash.
Keeping the Topic name from our example, the calls would be:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   source new_typesupport/install/setup.bash
   ros2 bag record /HelloWorldTopic

Launch your publisher application and you will see a discovery on the rosbag log:

.. code-block:: bash

   [INFO] [1644320308.422292205] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...

Rosbag will proceed to create a folder with an SQLite database inside. The path to this database file can be
used to replay the recorded messages. Open a subscriber application for the same topic and run:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   source new_typesupport/install/setup.bash
   ros2 bag play <path-to-db-file>

The recorded messages will be sent by rosbag at their original rate and the subscriber will receive them as such.
