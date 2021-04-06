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

``PushMode`` QoS Policy
^^^^^^^^^^^^^^^^^^^^^^^

By default, Fast DDS DataWriters are enabled using the ``push mode``.
This implies that the DataWriters start publishing and sending data following its own logic, disregarding if there are
any DataReaders matched in the same topic.
Using the :ref:`propertypolicyqos` DataWriters can be enabled using the ``pull mode``.
This entails that the DataWriters thus enabled will only send data upon the reception of acknack submessages coming from
the matched DataReaders.

Enabling ``pull mode`` implies that the |DataWriter::write-api| operation will only include the sample in the
DataWriter's History without notifying or sending the sample to any matched DataReader.
Periodically, DataWriters send its status by means of a heartbeat.
Upon reception of the heartbeat, DataReaders will be informed of samples on the DataWriter's History that are new.
DataReaders will answer with an acknack submessage which will trigger the sending of the sample on the DataWriter's
side.
Consequently, the publishing rate can be tuned setting the heartbeat period accordingly.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"pushMode"``
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
