.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _commonxml:

Common
------

The preceding XML profiles define some XML elements that are common to several profiles.
This section aims to explain these common elements.

*   :ref:`LocatorListType`
*   :ref:`PropertiesPolicyType`
*   :ref:`DurationType`
*   :ref:`TopicType`

    -   :ref:`hQos`
    -   :ref:`rLsQos`

*   :ref:`CommonQOS`

    -   :ref:`xml_durability`
    -   :ref:`xml_liveliness`
    -   :ref:`xml_partition`
    -   :ref:`xml_deadline`
    -   :ref:`xml_lifespan`
    -   :ref:`xml_disablepositiveacks`
    -   :ref:`xml_latencybudget`
    -   :ref:`xml_disableheartbeatpiggyback`

*   :ref:`Throughput`
*   :ref:`CommonAlloc`


.. _LocatorListType:

LocatorListType
^^^^^^^^^^^^^^^

It represents a list of |Locator_t-api|.
LocatorListType is used inside other configuration parameter labels that expect a list of locators,
for example, in ``<defaultUnicastLocatorList>``.
Therefore, LocatorListType is defined as a set of ``<locator>`` elements.
The ``<locator>`` element has a single child element that defines the transport protocol for which the locator is
defined. These are: ``<udpv4>``, ``<tcpv4>``, ``<udpv6>``, and ``<tcpv6>``.
The table presented below outlines each possible Locator's field.

.. note::

    :ref:`SHM transport <transport_sharedMemory_sharedMemory>` locators cannot be configured as they are
    automatically handled by SHM.

+---------------------+--------------------------------------------------------+--------------------+------------------+
| Name                | Description                                            | Values             | Default          |
+=====================+========================================================+====================+==================+
| ``<port>``          | RTPS port number of the locator. |br|                  | ``uint32_t``       | 0                |
|                     | *Physical port* in UDP,                                |                    |                  |
|                     | *logical port* in TCP.                                 |                    |                  |
+---------------------+--------------------------------------------------------+--------------------+------------------+
| ``<physical_port>`` | TCP's *physical port*.                                 | ``uint32_t``       | 0                |
+---------------------+--------------------------------------------------------+--------------------+------------------+
| ``<address>``       | IP address of the locator.                             | ``string``         | ""               |
|                     |                                                        | (IPv4/IPv6 format) |                  |
+---------------------+--------------------------------------------------------+--------------------+------------------+
| ``<unique_lan_id>`` | The LAN ID uniquely identifies the LAN the |br|        | ``string``         |                  |
|                     | locator belongs to (**TCPv4 only**).                   | (16 bytes)         |                  |
+---------------------+--------------------------------------------------------+--------------------+------------------+
| ``<wan_address>``   | WAN IPv4 address (**TCPv4 only**).                     | ``string``         | ``0.0.0.0``      |
|                     |                                                        | (IPv4 format)      |                  |
+---------------------+--------------------------------------------------------+--------------------+------------------+

**Example**

The following example shows the implementation of one locator of each transport protocol in
``<defaultUnicastLocatorList>``.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-LOCATOR-LIST<-->
    :end-before: <!--><-->

.. _PropertiesPolicyType:

PropertiesPolicyType
^^^^^^^^^^^^^^^^^^^^

PropertiesPolicyType defines the ``<propertiesPolicy>`` element.
It allows the user to define a set of generic properties inside a ``<properties>`` element.
It is useful at defining extended or custom configuration parameters.

+-----------------+---------------------------------------------------------------------+-------------+----------------+
| Name            | Description                                                         | Values      | Default        |
+=================+=====================================================================+=============+================+
| ``<name>``      | Name to identify the property.                                      | ``string``  |                |
+-----------------+---------------------------------------------------------------------+-------------+----------------+
| ``<value>``     | Property's value.                                                   | ``string``  |                |
+-----------------+---------------------------------------------------------------------+-------------+----------------+
| ``<propagate>`` | Indicates if it is going to be serialized along with the |br|       | ``bool``    | ``false``      |
|                 | object it belongs to.                                               |             |                |
+-----------------+---------------------------------------------------------------------+-------------+----------------+

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-PROPERTIES-POLICY<-->
    :end-before: <!--><-->

.. _DurationType:

DurationType
^^^^^^^^^^^^

DurationType expresses a period of time and it is commonly used inside other XML elements, such as in
``<leaseAnnouncement>`` or ``<leaseDuration>``.
A DurationType is defined by two mandatory elements ``<sec>`` plus ``<nanosec>``.
An infinite value can be specified by using the values :cpp:concept:`DURATION_INFINITY`,
:cpp:concept:`DURATION_INFINITE_SEC` and :cpp:concept:`DURATION_INFINITE_NSEC`.

+-----------------------+---------------------------------------------------------+-------------------+----------------+
| Name                  | Description                                             | Values            | Default        |
+=======================+=========================================================+===================+================+
| ``<sec>``             | Number of seconds.                                      | ``int32_t``       | 0              |
+-----------------------+---------------------------------------------------------+-------------------+----------------+
| ``<nanosec>``         | Number of nanoseconds.                                  | ``uint32_t``      | 0              |
+-----------------------+---------------------------------------------------------+-------------------+----------------+

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DURATION<-->
    :end-before: <!--><-->

.. _TopicType:

TopicType
^^^^^^^^^

The |Topic| name and data type are used to determine whether Datawriters and DataReaders can exchange messages.
Please refer to :ref:`dds_layer_topic` section for a a deeper explanation on the |Topic| class.

+-------------------------+-----------------------------------------------+--------------------------+-----------------+
| Name                    | Description                                   | Values                   | Default         |
+=========================+===============================================+==========================+=================+
| ``<kind>``              | It defines the Topic's key kind. See |br|     |                          |                 |
|                         | :ref:`dds_layer_definition_data_types`.       |                          |                 |
+-------------------------+-----------------------------------------------+--------------------------+-----------------+
| ``<name>``              | It defines the Topic's name. It must |br|     | ``string_255``           |                 |
|                         | be unique.                                    |                          |                 |
+-------------------------+-----------------------------------------------+--------------------------+-----------------+
| ``<dataType>``          | It references the Topic's data type.          | ``string_255``           |                 |
+-------------------------+-----------------------------------------------+--------------------------+-----------------+
| ``<historyQos>``        | It controls the behavior of *Fast DDS* |br|   | :ref:`hQos`              |                 |
|                         | when the value of an instance changes  |br|   |                          |                 |
|                         | before it is finally communicated to |br|     |                          |                 |
|                         | some of its existing DataReaders. |br|        |                          |                 |
+-------------------------+-----------------------------------------------+--------------------------+-----------------+
| ``<resourceLimitsQos>`` | It controls the resources that *Fast DDS*     | :ref:`rLsQos`            |                 |
|                         | |br| can use in order to meet the |br|        |                          |                 |
|                         | requirements imposed by the application |br|  |                          |                 |
|                         | and other QoS settings.                       |                          |                 |
+-------------------------+-----------------------------------------------+--------------------------+-----------------+

.. warning::

    The ``<kind>`` child element is only used if the Topic is defined using the *Fast DDS* RTPS-layer API, and will
    be ignored if the Topic is defined via the *Fast DDS* DDS-layer API.

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-TOPIC<-->
    :end-before: <!--><-->

.. _hQos:

HistoryQoS
""""""""""

It controls the behavior of *Fast DDS* when the value of an instance changes before it is finally
communicated to some of its existing DataReaders.
Please refer to :ref:`HistoryQosPolicyKind` for further information on HistoryQoS.

+-------------+---------------------------------------------------------+-----------------------+----------------------+
| Name        | Description                                             | Values                | Default              |
+=============+=========================================================+=======================+======================+
| ``<kind>``  | *Fast DDS* will only attempt to keep the latest values  | |KEEP_LAST-xml-api|   | |KEEP_LAST-xml-api|  |
|             | of the instance |br| and discard the older ones.        |                       |                      |
|             +---------------------------------------------------------+-----------------------+                      |
|             | *Fast DDS* will attempt to maintain and deliver all the | |KEEP_ALL-xml-api|    |                      |
|             | values of the instance |br| to existing DataReaders.    |                       |                      |
+-------------+---------------------------------------------------------+-----------------------+----------------------+
| ``<depth>`` | It must be consistent with the :ref:`rLsQos`            | ``uint32_t``          | 1                    |
|             | ``<max_samples_per_instance>`` |br|                     |                       |                      |
|             | element value. It must be verified that: |br|           |                       |                      |
|             | ``<depth>`` `<=` ``<max_samples_per_instance>``.        |                       |                      |
+-------------+---------------------------------------------------------+-----------------------+----------------------+

.. _rLsQos:

ResourceLimitsQos
"""""""""""""""""

It controls the resources that *Fast DDS* can use in order to meet the requirements imposed by the
application and other QoS settings.
Please refer to :ref:`ResourceLimitsQosPolicy` for further information on ResourceLimitsQos.

+--------------------------------+-----------------------------------------------------------+---------------+---------+
| Name                           | Description                                               | Values        | Default |
+================================+===========================================================+===============+=========+
| ``<max_samples>``              | It must verify that:                                      | ``uint32_t``  | 5000    |
|                                | ``<max_samples>`` `>=` ``<max_samples_per_instance>``.    |               |         |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<max_instances>``            | It defines the maximum number of instances.               | ``uint32_t``  | 10      |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<max_samples_per_instance>`` | It must verify that: :ref:`HistoryQos <hQos>`             | ``uint32_t``  | 400     |
|                                | ``<depth>`` `<=` ``<max_samples_per_instance>``.          |               |         |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<allocated_samples>``        | It controls the maximum number of samples to be stored.   | ``uint32_t``  | 100     |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<extra_samples>``            | The number of extra samples to allocate on the pool.      | ``uint32_t``  | 1       |
+--------------------------------+-----------------------------------------------------------+---------------+---------+

.. _CommonQOS:

QoS
^^^

The Quality of Service (QoS) is used to specify the behavior of the Service, allowing the user to define how each
|Entity| will behave.
Please refer to the :ref:`dds_layer_core_policy` section for more information on QoS.


+-----------------------------------+------------------------------------------+--------------------------------------+
| Name                              | Description                              | Values                               |
+===================================+==========================================+======================================+
| ``<durability>``                  | See :ref:`durabilityqospolicy`.          | :ref:`xml_durability`                |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<liveliness>``                  | See :ref:`livelinessqospolicy`.          | :ref:`xml_liveliness`                |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<reliability>``                 | See :ref:`reliabilityqospolicy`.         | :ref:`xml_reliability`               |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<partition>``                   | See :ref:`partitionqospolicy`.           | :ref:`xml_partition`                 |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<deadline>``                    | See :ref:`deadlineqospolicy`.            | :ref:`xml_deadline`                  |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<lifespan>``                    | See :ref:`lifespanqospolicy`.            | :ref:`xml_lifespan`                  |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<disablePositiveAcks>``         | See :ref:`disablepositiveacksqospolicy`. | :ref:`xml_disablepositiveacks`       |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<latencyBudget>``               | See :ref:`latencybudgetqospolicy`.       | :ref:`xml_latencybudget`             |
+-----------------------------------+------------------------------------------+--------------------------------------+
| ``<disable_heartbeat_piggyback>`` | See :ref:`disableheartbeatpiggyback`.    | :ref:`xml_disableheartbeatpiggyback` |
+-----------------------------------+------------------------------------------+--------------------------------------+

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-QOS<-->
    :end-before: <!--><-->


.. _xml_durability:

Durability
""""""""""

+------------+----------------------------+--------------------------------------+-------------------------------------+
| Name       | Description                | Values                               | Default                             |
+============+============================+======================================+=====================================+
| ``<kind>`` | See :ref:`durabilitykind`. | |VOLATILE-xml-api|                   | |VOLATILE-xml-api|                  |
|            |                            +--------------------------------------+                                     |
|            |                            | |TRANSIENT_LOCAL-xml-api|            |                                     |
|            |                            +--------------------------------------+                                     |
|            |                            | |TRANSIENT-xml-api|                  |                                     |
|            |                            +--------------------------------------+                                     |
|            |                            | |PERSISTENT-xml-api|                 |                                     |
+------------+----------------------------+--------------------------------------+-------------------------------------+

.. _xml_liveliness:

Liveliness
""""""""""

+---------------------------+---------------------------------+---------------------------------+----------------------+
| Name                      | Description                     | Values                          | Default              |
+===========================+=================================+=================================+======================+
| ``<kind>``                | See                             | |AUTOMATIC-xml-api|             | |AUTOMATIC-xml-api|  |
|                           | :ref:`livelinessqospolicykind`. +---------------------------------+                      |
|                           |                                 | |MANUAL_BY_PARTICIPANT-xml-api| |                      |
|                           |                                 +---------------------------------+                      |
|                           |                                 | |MANUAL_BY_TOPIC-xml-api|       |                      |
+---------------------------+---------------------------------+---------------------------------+----------------------+
| ``<lease_duration>``      | See :ref:`livelinessqospolicy`. | :ref:`DurationType`             | |c_TimeInfinite-api| |
+---------------------------+---------------------------------+---------------------------------+----------------------+
| ``<announcement_period>`` | See :ref:`livelinessqospolicy`. |                                 | |c_TimeInfinite-api| |
+---------------------------+---------------------------------+---------------------------------+----------------------+

.. _xml_reliability:

ReliabilityQosPolicy
""""""""""""""""""""

.. |max_block| replace:: ``<max_blocking_time>``

+------------------+-----------------------------------+-------------------------+-------------------------------------+
| Name             | Description                       | Values                  | Default                             |
+==================+===================================+=========================+=====================================+
| ``<kind>``       | See                               | |BEST_EFFORT-xml-api|   | DataReaders: |BEST_EFFORT-xml-api|  |
|                  | :ref:`reliabilityqospolicykind`.  +-------------------------+ |br|                                |
|                  |                                   | |RELIABLE-xml-api|      | DataWriters: |RELIABLE-xml-api|     |
+------------------+-----------------------------------+-------------------------+-------------------------------------+
| |max_block|      | See :ref:`reliabilityqospolicy`.  | :ref:`DurationType`     | 100 ms                              |
+------------------+-----------------------------------+-------------------------+-------------------------------------+

.. _xml_partition:

Partition
"""""""""

+---------------------------+-----------------------------------------------------------------------------+------------+
| Name                      | Description                                                                 | Values     |
+===========================+=============================================================================+============+
| ``<names>``               | It comprises a set of ``<name>`` elements containing the name of each       | ``<name>`` |
|                           | partition. |br| See :ref:`partitionqospolicy`.                              |            |
|                           |                                                                             |            |
+---------------------------+-----------------------------------------------------------------------------+------------+

.. _xml_deadline:

Deadline
""""""""
+---------------------------+----------------------------------+---------------------+---------------------------------+
| Name                      | Description                      | Values              | Default                         |
+===========================+==================================+=====================+=================================+
| ``<period>``              | See :ref:`deadlineqospolicy`.    | :ref:`DurationType` | |c_TimeInfinite-api|            |
+---------------------------+----------------------------------+---------------------+---------------------------------+

.. _xml_lifespan:

Lifespan
""""""""

+---------------------------+----------------------------------+---------------------+---------------------------------+
| Name                      | Description                      | Values              | Default                         |
+===========================+==================================+=====================+=================================+
| ``<duration>``            | See :ref:`lifespanqospolicy`.    | :ref:`DurationType` | |c_TimeInfinite-api|            |
+---------------------------+----------------------------------+---------------------+---------------------------------+

.. _xml_disablepositiveacks:

DisablePositiveAcks
"""""""""""""""""""

+--------------------+------------------------------------------+---------------------+--------------------------------+
| Name               | Description                              | Values              | Default                        |
+====================+==========================================+=====================+================================+
| ``<enabled>``      | See :ref:`disablepositiveacksqospolicy`. | ``bool``            | ``false``                      |
+--------------------+------------------------------------------+---------------------+--------------------------------+
| ``<duration>``     | See :ref:`disablepositiveacksqospolicy`. | :ref:`DurationType` | |c_TimeInfinite-api|           |
+--------------------+------------------------------------------+---------------------+--------------------------------+

.. _xml_latencybudget:

LatencyBudget
"""""""""""""

+---------------------------+-------------------------------------------------+---------------------+------------------+
| Name                      | Description                                     | Values              | Default          |
+===========================+=================================================+=====================+==================+
| ``<duration>``            | See :ref:`latencybudgetqospolicy`.              | :ref:`DurationType` | 0                |
+---------------------------+-------------------------------------------------+---------------------+------------------+

.. _xml_disableheartbeatpiggyback:

DisableHeartbeatPiggyback
"""""""""""""""""""""""""

+------+---------------------------------------+--------------+-----------+
| Name | Description                           | Values       | Default   |
+======+=======================================+==============+===========+
|      | See :ref:`disableheartbeatpiggyback`. | ``bool`` | ``false`` |
+------+---------------------------------------+--------------+-----------+

.. _Throughput:

Throughput Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

The ``<throughputController>`` element allows to limit the output bandwidth.
It contains two child elements which are explained in the following table.

+-----------------------+-----------------------------------------------------------+---------------+------------------+
| Name                  | Description                                               | Values        | Default          |
+=======================+===========================================================+===============+==================+
| ``<bytesPerPeriod>``  | Packet size in bytes that the throughput controller       | ``uint32_t``  | 4294967295 bytes |
|                       | will allow to send |br|                                   |               |                  |
|                       | in a given period.                                        |               |                  |
+-----------------------+-----------------------------------------------------------+---------------+------------------+
| ``<periodMillisecs>`` | Window of time in which no more than ``<bytesPerPeriod>`` | ``uint32_t``  | 0                |
|                       | bytes |br|                                                |               |                  |
|                       | are allowed.                                              |               |                  |
+-----------------------+-----------------------------------------------------------+---------------+------------------+

.. warning::

    This tag has been deprecated but does not have an equivalent tag yet.
    It will create a FIFO flow controller with the bandwidth limitation specified on this tag.
    See |FlowControllersQos| for more information.

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-THROUGHPUT-EXAMPLE<-->
    :end-before: <!--><-->

.. _historymemorypoliciesXML:

.. |MemManagment| replace:: ``<historyMemoryPolicy>``


HistoryMemoryPolicy
^^^^^^^^^^^^^^^^^^^

Indicates the way the memory is managed in terms of dealing with the CacheChanges of the :ref:`rtpsendpointqos`.

+----------------+--------------------------------------+-------------------------------------+------------------------+
| Name           | Description                          | Values                              | Default                |
+================+======================================+=====================================+========================+
| |MemManagment| |  Four different options as described | |PREALLOCATED-xml-api|              |                        |
|                |  |br| in                             +-------------------------------------+                        |
|                |  :ref:`memorymanagementpolicy`.      | |PREALLOCATED_WITH_REALLOC-xml-api| |                        |
|                |                                      +-------------------------------------+                        |
|                |                                      | |DYNAMIC-xml-api|                   | |PREALLOCATED-xml-api| |
|                |                                      +-------------------------------------+                        |
|                |                                      | |DYNAMIC_REUSABLE-xml-api|          |                        |
|                |                                      |                                     |                        |
|                |                                      |                                     |                        |
+----------------+--------------------------------------+-------------------------------------+------------------------+

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-HISTORYMEMORYPOLICY-EXAMPLE<-->
    :end-before: <!--><-->
    :dedent: 4

.. _CommonAlloc:

Allocation Configuration
^^^^^^^^^^^^^^^^^^^^^^^^

The ``<allocation>`` element allows to control the allocation behavior of internal collections for which the number
of elements depends on the number of entities in the system.
For instance, there are collections inside a DataWriter which depend on the number of DataReaders matching with it.
Please refer to :ref:`participantresourcelimitsqos` for a detailed documentation on DomainParticipant allocation,
and to :ref:`realtime-allocations` for detailed information on how to tune allocation related parameters.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<initial>``
     - Number of elements for which space is initially allocated.
     - ``uint32_t``
     - 0
   * - ``<maximum>``
     - Maximum number of elements for which space will be allocated.
     - ``uint32_t``
     - 0 (Means no limit)
   * - ``<increment>``
     - Number of new elements that will be allocated when more space is |br| necessary.
     - ``uint32_t``
     - 1
