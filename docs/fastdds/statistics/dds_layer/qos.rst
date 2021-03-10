.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _statistics_qos:

Statistics recommended QoS
==========================

Although the statistics DataWriters can be enabled using any valid QoS profile, the recommended profile is presented
below.
Also, the DataReaders created by the user to receive the data being published by the statistics DataWriters can use any
compatible QoS profile.
However, a recommended DataReader QoS profile is also provided.

.. _statistics_datawriter_qos:

Statistics DataWriter recommended QoS
-------------------------------------

The following table shows the recommended |DataWriterQos-api| profile for enabling the statistics DataWriters.
This profile enables the ``pull mode`` on the statistics DataWriters (``pushMose=false``).
This entails that the DataWriters will only send information upon the reception of acknack submessages sent by the
monitoring DataReader.

.. list-table::
   :header-rows: 1
   :align: left

   * - Qos Policy
     - Value
   * - |ReliabilityQosPolicyKind-api|
     - |RELIABLE_RELIABILITY_QOS-api|
   * - |DurabilityQosPolicyKind-api|
     - |TRANSIENT_LOCAL_DURABILITY_QOS-api|
   * - |PublishModeQosPolicyKind-api|
     - |ASYNCHRONOUS_PUBLISH_MODE-api|
   * - |HistoryQosPolicyKind-api|
     - |KEEP_LAST_HISTORY_QOS-api|
   * - |history_depth-api|
     - 100
   * - Push mode
     - ``FALSE``

Statistics DataReader recommended QoS
-------------------------------------

The following table shows the recommended |DataReaderQos-api| profile for creating the monitoring DataReaders.

.. list-table::
   :header-rows: 1
   :align: left

   * - Qos Policy
     - Value
   * - |ReliabilityQosPolicyKind-api|
     - |RELIABLE_RELIABILITY_QOS-api|
   * - |DurabilityQosPolicyKind-api|
     - |TRANSIENT_LOCAL_DURABILITY_QOS-api|
   * - |HistoryQosPolicyKind-api|
     - |KEEP_LAST_HISTORY_QOS-api|
   * - |history_depth-api|
     - 100
