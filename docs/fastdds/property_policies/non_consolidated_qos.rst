.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _property_policies_non_consolidated_qos:

Non consolidated QoS
--------------------

The :ref:`property_policies` are used to develop new :ref:`eprosima_extensions` QoS.
Before consolidating a new QoS Policy, it is usually set using this generic QoS Policy.
Consequently, this section is prone to frequent updates so the user is advised to check latest changes after upgrading
to a different release version.

.. _push_mode_qos_policy:

``Push Mode`` QoS Policy
^^^^^^^^^^^^^^^^^^^^^^^

By default, Fast DDS DataWriters are enabled using the ``push mode``.
This implies they will send new samples to matched readers inmediately after adding them to their queue.
For writers that produce non periodic bursts of data, this may imply saturating the network with a lot of packets,
increasing the possibility of losing them on unreliable (i.e. UDP) transports.
Depending on their QoS, DataReaders may also have to ignore some received samples, so they will have to be resent.

Configuring the DataWriters on ``pull mode`` offers an alternative by letting each reader pace its own data stream.
It works by notifying the reader what it is missing, and wait for it to request only as much as it can handle.
At the cost of greater latency, this model can deliver reliability while using far fewer packets than in ``push mode``.

DataWriters periodically announce the state of their queue by means of a heartbeat.
Upon reception of the heartbeat, DataReaders will request the DataWriter to send the samples they want to process.
Consequently, the publishing rate can be tuned setting the heartbeat period accordingly.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"fastdds.push_mode"``
     - ``"true"``/``"false"``
     - ``"true"``

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // PULL_MODE_DATAWRITER                                                                             |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->PULL_MODE_DATAWRITER<-->                                                                       |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

.. note::
    Communication to readers running on the same process will always use ``push mode``

.. warning::
    It is inconsistent to enable the ``pull mode`` and also set the |ReliabilityQosPolicyKind-api| to
    |BEST_EFFORT_RELIABILITY_QOS-api|.

    It is inconsistent to enable the ``pull mode`` and also set the |WriterTimes::heartbeatPeriod-api| to
    |c_TimeInfinite-api|.


Unique network flows QoS Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This section is still under work.

Statistics Module Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This section is still under work.
