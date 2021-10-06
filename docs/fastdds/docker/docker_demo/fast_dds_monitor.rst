.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _fast_dds_monitor:

Fast DDS Monitor
================

This Docker image contains **eProsima Fast DDS Monitor**. It' s a graphical desktop application aimed at monitoring DDS 
environments deployed using the eProsima Fast DDS library. Thus, the user can monitor in real time the status 
of publication/subscription communications between DDS entities. They can also choose from a wide variety of 
communication parameters to be measured (latency, throughput, packet loss, etc.), as well as record and compute 
in real time statistical measurements on these parameters (mean, variance, standard deviation, etc.).

eProsima Fast DDS Monitor documentation can be located `here <https://fast-dds-monitor.readthedocs.io/en/latest/index.html>`_.

To load this image into your Docker repository, from a terminal run

.. code-block:: bash

   $ docker load -i ubuntu-fastdds-monitor:<FastDDS-Version>.tar

You can run this Docker container as follows

.. code-block:: bash

   $ xhost local:root
   $ docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
      ubuntu-fastdds-monitor:<FastDDS-Version>
