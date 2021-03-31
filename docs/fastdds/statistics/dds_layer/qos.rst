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
This profile enables the ``pull mode`` on the statistics DataWriters (please refer to :ref:`push_mode_qos_policy`).
This entails that the DataWriters will only send information upon the reception of acknack submessages sent by the
monitoring DataReader.
The recommended profile can be accessed through constant |STATISTICS_DATAWRITER_QOS-api|.

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
   * - |PropertyPolicyQos-api| name/value
     - ``"pushMode"``/``"false"``

Statistics DataReader recommended QoS
-------------------------------------

The following table shows the recommended |DataReaderQos-api| profile for creating the monitoring DataReaders.
The recommended profile can be accessed through constant |STATISTICS_DATAREADER_QOS-api|.

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
