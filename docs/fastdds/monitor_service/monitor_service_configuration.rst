.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _monitor_service_configuration:

Monitor Service Configuration
=============================

The :ref:`monitor_service` can be programmatically enabled in both :ref:`dds_layer` and :ref:`rtps_layer`
through the |DomainParticipant::enable-monitor-service-api| and |DomainParticipant::disable-monitor-service-api| calls.
In addition, leveraging the |PropertyPolicyQos-api| there is new ``Property`` defined
for the purpose: ``fastdds.enable_monitor_service``.

The following table depicts the different ways in which the monitor service can be enabled or disabled:


+----------------------------------------------------------+
| **C++ API**                                              |
+----------------------------------------------------------+
| .. literalinclude:: ../../../code/DDSCodeTester.cpp      |
|        :language: c++                                    |
|        :dedent: 8                                        |
|        :start-after: // FASTDDS_MONITOR_SERVICE_API      |
|        :end-before: //!                                  |
+----------------------------------------------------------+
| **C++ Property**                                         |
+----------------------------------------------------------+
| .. literalinclude:: ../../../code/DDSCodeTester.cpp      |
|        :language: c++                                    |
|        :dedent: 8                                        |
|        :start-after: // FASTDDS_MONITOR_SERVICE_PROPERTY |
|        :end-before: //!                                  |
+----------------------------------------------------------+
| **XML**                                                  |
+----------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml               |
|    :language: xml                                        |
|    :start-after: <!-->DDS_MONITOR_SERVICE<-->            |
|    :end-before: <!--><-->                                |
+----------------------------------------------------------+
| **Environment Variable Linux**                           |
+----------------------------------------------------------+
| .. code-block:: bash                                     |
|                                                          |
|    export FASTDDS_STATISTICS="MONITOR_SERVICE_TOPIC"     |
+----------------------------------------------------------+
| **Environment Variable Windows**                         |
+----------------------------------------------------------+
| .. code-block:: bash                                     |
|                                                          |
|    set FASTDDS_STATISTICS=MONITOR_SERVICE_TOPIC          |
+----------------------------------------------------------+

Endpoints QoS
^^^^^^^^^^^^^

For any consumer application of the :ref:`monitor_service`, the following endpoint QoS
of the :ref:`monitor_service_status_topic` should be taken into consideration:

+-------------------------+-------------------------------+------------------------------------+
|       **Endpoint**      |          **QoS**              |  **Value**                         |
+-------------------------+-------------------------------+------------------------------------+
|  MONITOR_STATUS_WRITER  | |ReliabilityQosPolicyKind-api|||RELIABLE_RELIABILITY_QOS-api|      |
|                         +-------------------------------+------------------------------------+
|                         | |HistoryQosPolicyKind-api|    ||KEEP_LAST_HISTORY_QOS-api| 1       |
|                         +-------------------------------+------------------------------------+
|                         | |DurabilityQosPolicyKind-api| ||TRANSIENT_LOCAL_DURABILITY_QOS-api||
+-------------------------+-------------------------------+------------------------------------+
