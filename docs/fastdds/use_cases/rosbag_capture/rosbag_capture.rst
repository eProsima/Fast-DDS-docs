.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _rosbag_capture:

Rosbag2 Record and Replay
==========================

Rosbag2 is a ROS application that can be used to capture DDS messages and store them on an sqlite database. This allows 
inspecting and replaying said messages at a later time. 

Rosbag2 interactions with a non-ROS2 Fast DDS application
---------------------------------------------------------

Using rosbag2 to capture traffic between ROS talkers and listeners is straightforward. Using it to record and replay
messages sent by Fast DDS participants outside ROS2 ecosystem requires some modifications. 

Prerequisites
^^^^^^^^^^^^^
Being a ROS2 application, to install rosbag requires to have the ROS2 repository for the ROS2 distribution of choice
set up in your sources file. Instructions on how to do so can be found on the `ROS2 documentation
<https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html#add-the-ros-2-apt-repository>`_

Rosbag2 can then be installed following the instructions on their `Github repository
<https://github.com/ros2/rosbag2>`_

IDL files
^^^^^^^^^
IDL files used to generate types for the application need to have the types nested inside both the module name and an idl 
module, in that order, for instance:

.. code-block:: bash

   module helloworld_type{
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
Create colcon package

.. code-block:: bash

   source /opt/ros/galactic/setup.bash
   ros2 pkg create --build-type ament_cmake new_typesupport

This will create a new_typesupport folder. Inside this folder create an *idl* folder. Place the IDL files inside.

Add the following to your CMakeLists.txt file just before the ament_package() statement

.. code-block:: cmake

   find_package(rosidl_default_generators REQUIRED)

   set(idl_files
     "idl/<idl_file1>.idl"
   )

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${idl_files}
   )

Add the following to the package.xml file:

.. code-block:: xml

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

Build the package:

.. code-block:: bash

   colcon build

Fast DDS Application modifications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Generate the types using Fast-DDS-Gen:

.. code-block:: bash

   fastddsgen -typeros2 <idl_file1>.idl

ROS2 adds the token "rt/" on all topics so that needs to be added to the topic name.

Retrieve the DataTypeName using the type->getName() accessor instead of hardcoding it.

Rosbag Record and Play
^^^^^^^^^^^^^^^^^^^^^^

Source the type support package you created earlier and then launch rosbag2.

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   source new_typesupport/install/setup.bash
   ros2 bag record /TopicNameWithoutRT

Launch your publisher application and you will see a discovery on the rosbag log:

[INFO] [1644320308.422292205] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...

Rosbag creates a folder with an sqlite database inside. You can use the path to that file to replay the
recorded messages. If you open a subscriber application and run:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   source new_typesupport/install/setup.bash
   ros2 bag play <path-to-db-file>

You will receive the recorded messages at the same rate they were sent. 