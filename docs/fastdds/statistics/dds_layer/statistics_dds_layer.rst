.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _statistics_dds_layer:

Statistics Module DDS Layer
===========================

This section explains the extended DDS API provided for the *Statistics Module*.
First, the Statistics Topic list is presented together with the corresponding collected data.
Next, the methods to enable/disable the corresponding DataWriters are explained.
Finally, the recommended QoS for enabling the DataWriters and creating the user's
DataReaders that subscribe to the Statistics topics are described.

.. toctree::
   :maxdepth: 2

   /fastdds/statistics/dds_layer/topic_names.rst
   /fastdds/statistics/dds_layer/domainParticipant.rst
   /fastdds/statistics/dds_layer/qos.rst

