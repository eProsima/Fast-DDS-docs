.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _statistics_domainparticipant:

Statistics Domain Participant
=============================

In order to start collecting data in one of the statistics topics (:ref:`statistics_topic_names`), the corresponding
statistics DataWriter should be enabled.
In fact, *Fast DDS Statistics module* can be enabled and disabled at runtime.
For this purpose, *Fast DDS Statistics module* exposes an extended DDS |StatisticsDomainParticipant-api| API:

Enable statistics DataWriters
-----------------------------

Statistics DataWriters are enabled using the method |enable_statistics_datawriter|.
This method requires as parameters:

* Name of the statistics topic to be enabled (see :ref:`statistics_topic_names` for the statistics topic list).
* DataWriter QoS profile (see :ref:`statistics_datawriter_qos` for the recommended profile).

Disable statistics DataWriters
------------------------------

Statistics DataWriters are disabled using the method |disable_statistics_datawriter|.
This method requires as parameter:

* Name of the statistics topic to be disabled (see :ref:`statistics_topic_names` for the statistics topic list).

Obtain pointer to the extended |StatisticsDomainParticipant-api| class
----------------------------------------------------------------------

The |DomainParticipant-api| is created using the |DomainParticipantFactory::create_participant-api| provided by the
|DomainParticipantFactory-api|.
This method returns a pointer to the DDS standard DomainParticipant created.
In order to obtain the pointer to the child |StatisticsDomainParticipant-api| which extends the DDS API, the
``static`` method |statistics_narrow| is provided.

.. _auto_enabling_statistics_datawriters:

Automatically enabling statistics DataWriters
---------------------------------------------

The statistics DataWriters can be directly enabled using the |DomainParticipantQos|
|DomainParticipantQos::properties-api| ``fastdds.statistics``.
The value of this property is a semicolon separated list containing the
:ref:`statistics topic name aliases<statistics_topic_names>` of those DataWriters that the user wants to enable.
The property can be set programmatically, loading an XML file and setting the :ref:`env_vars_fastdds_statistics`
environment variable as shown in the following examples:

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // FASTDDS_STATISTICS_MODULE                                                                        |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->FASTDDS_STATISTICS_MODULE<-->                                                                  |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+
| **Environment Variable Linux**                                                                                       |
+----------------------------------------------------------------------------------------------------------------------+
| .. code-block:: bash                                                                                                 |
|                                                                                                                      |
|    export FASTDDS_STATISTICS=HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC           |
+----------------------------------------------------------------------------------------------------------------------+
| **Environment Variable Windows**                                                                                     |
+----------------------------------------------------------------------------------------------------------------------+
| .. code-block:: bash                                                                                                 |
|                                                                                                                      |
|    set FASTDDS_STATISTICS=HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC              |
+----------------------------------------------------------------------------------------------------------------------+

For more information about the environment variable, please refer to :ref:`env_vars_fastdds_statistics`.
This method is compatible with setting also the property through code or XML.
The statistics DataWriters that will be enabled when the |DomainParticipant-api| is enabled would be the union between
those specified in the |DomainParticipantQos::properties-api| ``fastdds.statistics`` and those included with the
environment variable.

Be aware that automatically enabling the statistics DataWriters using these methods implies using the recommended
QoS profile |STATISTICS_DATAWRITER_QOS-api|. For more information, please refer to :ref:`statistics_datawriter_qos`.

.. warning::
    Currently, only the following :ref:`statistics topics <statistics_topic_names>` have been implemented:

        * |RTPS_SENT_TOPIC|

        * |HEARTBEAT_COUNT_TOPIC|

        * |ACKNACK_COUNT_TOPIC|

        * |NACKFRAG_COUNT_TOPIC|

        * |GAP_COUNT_TOPIC|

        * |DATA_COUNT_TOPIC|

        * |DISCOVERY_TOPIC|

    Other statistics topics will be implemented in further releases.
