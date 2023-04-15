.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _monitor_service_configuration:

Monitor Service Configuration
=============================

The :ref:`monitor_service` can be programatically enabled in both :ref:`dds_layer` and :ref:`rtps_layer`
through the |DomainParticipant::enable-monitor-service-api| and |DomainParticipant::disable-monitor-service-api| calls.
In addition, leveraging the |PropertyPolicyQos-api| there is new ``Property`` defined for the purpose: ``fastdds.enable_monitor_service``.

The following table depicts the different ways in which the monitor service can be enabled or disbled:

Property Policy
^^^^^^^^^^^^^^^

.. tabs::

  .. tab:: C++ API

    .. literalinclude:: ../../../code/DDSCodeTester.cpp
       :language: c++
       :dedent: 8
       :start-after: // FASTDDS_MONITOR_SERVICE_API
       :end-before: //!

  .. tab:: C++ Property

    .. literalinclude:: ../../../code/DDSCodeTester.cpp
       :language: c++
       :dedent: 8
       :start-after: // FASTDDS_MONITOR_SERVICE_PROPERTY
       :end-before: //!

  .. tab:: XML

    .. literalinclude:: ../../../code/XMLTester.xml
       :language: xml
       :start-after: <!-->DDS_MONITOR_SERVICE
       :end-before: <!--><-->

Endpoints QoS
^^^^^^^^^^^^^

For any client of the :ref:`monitor_service`, the following endpoints' QoS for each
of the :ref:`monitor_service_topics` should be taken into consideration:

+-------------------------+-------------------------------+------------------------------------+
|       **Endpoint**      |          **QoS**              |  **Value**                         |
+-------------------------+-------------------------------+------------------------------------+
|     MONITOR_EV_WRITER   | |ReliabilityQosPolicyKind-api|||RELIABLE_RELIABILITY_QOS-api|      |
|                         +-------------------------------+------------------------------------+
|                         | |HistoryQosPolicyKind-api|    ||KEEP_LAST_HISTORY_QOS-api| 1       |
|                         +-------------------------------+------------------------------------+
|                         | |DurabilityQosPolicyKind-api| ||TRANSIENT_LOCAL_DURABILITY_QOS-api||
+-------------------------+-------------------------------+------------------------------------+
|     MONITOR_REQ_READER  | |ReliabilityQosPolicyKind-api|||RELIABLE_RELIABILITY_QOS-api|      |
|                         +-------------------------------+------------------------------------+
|                         | |DurabilityQosPolicyKind-api| ||TRANSIENT_LOCAL_DURABILITY_QOS-api||
+-------------------------+-------------------------------+------------------------------------+
|     MONITOR_RES_WRITER  | |ReliabilityQosPolicyKind-api|||RELIABLE_RELIABILITY_QOS-api|      |
|                         +-------------------------------+------------------------------------+
|                         | |DurabilityQosPolicyKind-api| ||TRANSIENT_LOCAL_DURABILITY_QOS-api||
+-------------------------+-------------------------------+------------------------------------+



