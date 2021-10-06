.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _fast_dds_hello_world:

Fast DDS Hello World
====================

This Docker image contains **eProsima Fast DDS** bundled with a simple Hello World application that can be used as a quick 
demonstration of the communication capabilities between a Publisher and a Subscriber using Fast DDS. 

To load this image into your Docker repository, from a terminal run

.. code-block:: bash

   $ docker load -i ubuntu-fast-dds-helloworld:<FastDDS-Version>.tar

You can run this Docker container as follows

.. code-block:: bash

   $ docker run -it ubuntu-fast-dds-helloworld:<FastDDS-Version>

This will perform a Publisher/Subscriber match and start sending samples. This example is not constrained to the 
current instance. It's possible to run several instances of this container to check the communication between them.