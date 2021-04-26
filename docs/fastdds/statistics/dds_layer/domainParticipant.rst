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

The following example shows how to use the Statistics module extended DDS API:

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: // ENABLE_DISABLE_STATISTICS_DATAWRITER
   :end-before: //!--
   :dedent: 8

.. _auto_enabling_statistics_datawriters:

Automatically enabling statistics DataWriters
---------------------------------------------

The statistics DataWriters can be directly enabled using the |DomainParticipantQos|
|DomainParticipantQos::properties-api| ``fastdds.statistics``.
The value of this property is a semicolon separated list containing the
:ref:`statistics topic name aliases<statistics_topic_names>` of those DataWriters that the user wants to enable.
The property can be set either programmatically or loading an XML file.
If the property is set in both ways, the priority would depend on the QoS profile provided to
|DomainParticipantFactory::create_participant-api|.
If the method is called using |PARTICIPANT_QOS_DEFAULT-api| the values coming from the XML file will be used if the
``is_default_profile`` option is set to ``true`` (:ref:`domainparticipantattributes`).
The XML file is also used when calling |DomainParticipantFactory::create_participant_with_profile-api| with a valid
participant profile defined within the XML file.
Otherwise, the property value set programmatically would be taken into account.

Another way compatible with the previous one is setting the :ref:`env_vars_fastdds_statistics` environment variable.
The statistics DataWriters that will be enabled when the |DomainParticipant-api| is enabled would be the union between
those specified in the |DomainParticipantQos::properties-api| ``fastdds.statistics`` and those included with the
environment variable.

The following examples show how to use all the previous methods:

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

Be aware that automatically enabling the statistics DataWriters using all these methods implies using the recommended
QoS profile |STATISTICS_DATAWRITER_QOS-api|. For more information, please refer to :ref:`statistics_datawriter_qos`.

.. warning::
    Currently, the following :ref:`statistics topics <statistics_topic_names>` have not been implemented yet. They will
    be available in future releases:

        * |HISTORY_LATENCY_TOPIC|

        * |NETWORK_LATENCY_TOPIC|

        * |PUBLICATION_THROUGHPUT_TOPIC|

        * |SUBSCRIPTION_THROUGHPUT_TOPIC|

        * |RTPS_LOST_TOPIC|

        * |RESENT_DATAS_TOPIC|

        * |SAMPLE_DATAS_TOPIC|

        * |PDP_PACKETS_TOPIC|

        * |EDP_PACKETS_TOPIC|

        * |PHYSICAL_DATA_TOPIC|
