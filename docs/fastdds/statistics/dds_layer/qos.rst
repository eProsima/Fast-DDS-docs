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
This profile enables the ``pull mode`` :ref:`operating mode <push_mode_qos_policy>` on the statistics DataWriters.
This entails that the DataWriters will only send information upon the reception of acknack submessages sent by the
monitoring DataReader.
This QoS profile is always used when the statistics DataWriters are
:ref:`auto-enabled <auto_enabling_statistics_datawriters>`.
The recommended profile can be accessed through the constant |STATISTICS_DATAWRITER_QOS-api|.

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
   * - |PublishModeQosPolicy::flow_ctrl_name-api|
     - |FASTDDS_STATISTICS_FLOW_CONTROLLER_DEFAULT-api|
   * - |HistoryQosPolicyKind-api|
     - |KEEP_LAST_HISTORY_QOS-api|
   * - |history_depth-api|
     - 1
   * - |PropertyPolicyQos-api| name = value
     - ``"fastdds.push_mode"`` = ``"false"``

.. _statistics_datareader_qos:

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
   * - |MemoryManagementPolicy-api|
     - |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|

.. _statistics_qos_warning:

Statistics QoS Troubleshooting
------------------------------

Some statistics intensive deployments may face statistics' data loss due to the default QoS settings for the statistics
DataWriter being insufficient to hold the amount of samples pending to be sent leading to sample overwrites.
This can be mitigated or avoided altogether by increasing the value of the statistics DataWriters's History
depth QoS settings.

Keep in mind that altering this value will increase memory usage.

Statistics entities' QoS configuration via profiles is detailed at the |StatisticsDomainParticipant| page.


