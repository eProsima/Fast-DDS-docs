.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _fast_dds_shapes_demo:

Fast DDS Shapes Demo
====================

This Docker image contains **eProsima Fast-DDS Shapes Demo**. eProsima Shapes Demo is an application in which Publishers and 
Subscribers are shapes of different colors and sizes moving on a board. Each shape refers to its own topic: Square, Triangle 
or Circle. A single instance of the eProsima Shapes Demo can publish on or subscribe to several topics at a time.

You can read more about this application and its usage on the 
`Shapes Demo documentation page <https://eprosima-shapes-demo.readthedocs.io/>`_.

To load this image into your Docker repository, from a terminal run

.. code-block:: bash

   $ docker load -i ubuntu-fastdds-shapesdemo:<FastDDS-Version>.tar

You can run this Docker container as follows

.. code-block:: bash

   $ xhost local:root
   $ docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
      ubuntu-fastdds-shapesdemo:<FastDDS-Version>
