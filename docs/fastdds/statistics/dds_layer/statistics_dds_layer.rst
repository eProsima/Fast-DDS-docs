.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _statistics_dds_layer:

Statistics Module DDS Layer
===========================

This section explains the extended DDS API provided for the *Statistics Module*.
First, the Statistics Topic List is presented together with the corresponding collected data.
Next, the methods to enable/disable the corresponding DataWriters are explained.
Then, the recommended QoS for enabling the DataWriters and creating the user's DataReaders that subscribe to the
Statistics topics are described.
Finally, a guide on how to overcome common problems when using the module are presented.

.. toctree::
   :maxdepth: 2

   /fastdds/statistics/dds_layer/topic_names.rst
   /fastdds/statistics/dds_layer/domainParticipant.rst
   /fastdds/statistics/dds_layer/qos.rst
   /fastdds/statistics/dds_layer/troubleshooting.rst
