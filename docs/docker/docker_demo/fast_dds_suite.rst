.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fast_dds_suite:

Fast DDS Suite
==============

This Docker image contains the complete Fast DDS suite. This includes:

- :ref:`eProsima Fast DDS libraries and examples <fast_dds_suite_examples>`: Fast DDS libraries bundled with several examples that showcase
  a variety of caabilities of eProsima's Fast DDS implementation.

- :ref:`Shapes Demo <fast_dds_suite_shapes_demo>`: eProsima Shapes Demo is an application in which Publishers and Subscribers are shapes of
  different colors and sizes moving on a board. Each shape refers to its own topic: Square, Triangle or Circle.
  A single instance of the eProsima Shapes Demo can publish on or subscribe to several topics at a time.

  You can read more about this application on the `Shapes Demo documentation page <https://eprosima-shapes-demo.readthedocs.io/>`_.

- :ref:`Fast DDS Monitor <fast_dds_suite_monitor>`: eProsima Fast DDS Monitor is a graphical desktop application aimed at monitoring DDS
  environments deployed using the *eProsima Fast DDS* library. Thus, the user can monitor in real time the status
  of publication/subscription communications between DDS entities. They can also choose from a wide variety of
  communication parameters to be measured (latency, throughput, packet loss, etc.), as well as record and compute
  in real time statistical measurements on these parameters (mean, variance, standard deviation, etc.).
 
  You can read more about this application on the `Fast DDS Monitor documentation page
  <https://eprosima-shapes-demo.readthedocs.io/>`_.


To load this image into your Docker repository, from a terminal run

.. code-block:: bash

   $ docker load -i ubuntu-fastdds-suite:<FastDDS-Version>.tar

You can run this Docker container as follows

.. code-block:: bash

   $ xhost local:root
   $ docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
      ubuntu-fastdds-suite:<FastDDS-Version>

From the resulting Bash Shell you can run each feature.

.. _fast_dds_suite_examples:

 * **Fast DDS Examples**

Included in this Docker container is a set of binary examples that showcase several functionalities of the Fast DDS libraries.
More information can be found :ref:`here <fast_dds_examples_docker>`.

.. _fast_dds_suite_shapes_demo:

 * **Shapes Demo**

To launch the Shapes Demo, from a terminal run

 .. code-block:: bash

   $ ShapesDemo

eProsima Shapes Demo usage information can be found on the `Shapes Demo documentation
<https://eprosima-shapes-demo.readthedocs.io/en/latest/first_steps/first_steps.html>`_.

.. _fast_dds_suite_monitor:

 * **Fast DDS Monitor**

To launch the Fast DDS Monitor, from a terminal run

 .. code-block:: bash

   $ fastdds_monitor

eProsima Fast DDS Monitor User Manual can be located on the `Fast DDS Monitor documentation
<https://fast-dds-monitor.readthedocs.io/en/latest/rst/user_manual/initialize_monitoring.html>`_.







