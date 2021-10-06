.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _fast_dds_rtps_docker:

Fast DDS Examples
=================

This Docker image contains **eProsima Fast DDS** bundled with several example applications that can be used to demonstrate
some of Fast DDS capabilities. 

To load this image into your Docker repository, from a terminal run 

.. code-block:: bash

   $ docker load -i ubuntu-fast-rtps:<FastDDS-Version>.tar

This Docker Image can be launched with the absolute path of the example you want to launch. For instance, to run the Benchmark 
example you could do so by running

.. code-block:: bash

   $ docker run -it ubuntu-fast-rtps:<FastDDS-Version> /usr/local/examples/C++/Benchmark/bin/Benchmark subscriber udp
   $ docker run -it ubuntu-fast-rtps:<FastDDS-Version> /usr/local/examples/C++/Benchmark/bin/Benchmark publisher udp
