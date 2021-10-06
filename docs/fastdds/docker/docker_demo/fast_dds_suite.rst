.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _fast_dds_suite:

Fast DDS Suite
==============

This Docker image contains the complete Fast DDS suite. This includes:

- :ref:`eProsima Fast DDS libraries and examples <fast_dds_suite_examples>`
- :ref:`Hello World Example <fast_dds_suite_hello_world>`: Simple application that can showcase the minimal DDS communication capabilities between
  a Publisher and a Subscriber, either both deployed in the same container on between different hosts. 
- :ref:`Shapes Demo <fast_dds_suite_hello_world>`: eProsima Shapes Demo is an application in which Publishers and Subscribers are shapes of 
  different colors and sizes moving on a board. Each shape refers to its own topic: Square, Triangle or Circle. 
  A single instance of the eProsima Shapes Demo can publish on or subscribe to several topics at a time.
- :ref:`Fast DDS Monitor <fast_dds_suite_monitor>`: eProsima Fast DDS Monitor is a graphical desktop application aimed at monitoring DDS 
  environments deployed using the eProsima Fast DDS library. Thus, the user can monitor in real time the status 
  of publication/subscription communications between DDS entities. They can also choose from a wide variety of 
  communication parameters to be measured (latency, throughput, packet loss, etc.), as well as record and compute 
  in real time statistical measurements on these parameters (mean, variance, standard deviation, etc.).

To load this image into your Docker repository, from a terminal run

.. code-block:: bash

   $ docker load -i ubuntu-fastdds-suite:<FastDDS-Version>.tar

You can run this Docker container as follows

.. code-block:: bash

   $ xhost local:root
   $ docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
      ubuntu-fastdds-suite:<FastDDS-Version>

From the resulting Bash Shell you can run each feature. 

.. _fast_dds_suite_shapes_demo:

 * **Shapes Demo**

To launch the Shapes Demo, from a terminal run

 .. code-block:: bash

   $ ShapesDemo

You can read more about this application and its usage on the 
`Shapes Demo documentation page <https://eprosima-shapes-demo.readthedocs.io/>`_.

.. _fast_dds_suite_monitor:

 * **Fast DDS Monitor**

 .. code-block:: bash

   $ fastdds_monitor

eProsima Fast DDS Monitor documentation can be located `here <https://fast-dds-monitor.readthedocs.io/en/latest/index.html>`_.

.. _fast_dds_suite_examples:

 * **Fast DDS Examples**

Included in this Docker container is a set of binary examples that showcase several functionalities of the Fast DDS libraries.
These examples path can be accessed from a terminal by typing 

 .. code-block:: bash

   $ goToExamples


.. _fast_dds_suite_hello_world:

 * **Hello World Example**

 .. code-block:: bash
   
   $ goToExamples
   $ cd HelloWorldExample/bin 
   $ tmux new-session "./HelloWorldExample publisher 0 1000" \; \
   split-window "./HelloWorldExample subscriber" \; \
   select-layout even-vertical

 This example can also be run from several different containers. From each you can run:
 
 .. code-block:: bash
   
   $ goToExamples
   $ cd HelloWorldExample/bin 
   $ ./HelloWorldExample publisher

 or

 .. code-block:: bash

   $ goToExamples
   $ cd HelloWorldExample/bin  
   $ ./HelloWorldExample subscriber





