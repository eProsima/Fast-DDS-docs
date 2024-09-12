.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _commonxml:

Common
------

The preceding XML profiles define some XML elements that are common to several profiles.
This section aims to explain these common elements.

*   :ref:`LocatorListType`
*   :ref:`externalLocatorListType`
*   :ref:`PropertiesPolicyType`
*   :ref:`DurationType`
*   :ref:`TopicType`

    -   :ref:`hQos`
    -   :ref:`rLsQos`

*   :ref:`CommonQOS`

    -   :ref:`xml_datasharing`
    -   :ref:`xml_deadline`
    -   :ref:`xml_disableheartbeatpiggyback`
    -   :ref:`xml_disablepositiveacks`
    -   :ref:`xml_durability`
    -   :ref:`xml_groupData`
    -   :ref:`xml_latencybudget`
    -   :ref:`xml_lifespan`
    -   :ref:`xml_liveliness`
    -   :ref:`xml_ownership`
    -   :ref:`xml_ownershipstrength`
    -   :ref:`xml_partition`
    -   :ref:`xml_publishmode`
    -   :ref:`xml_reliability`
    -   :ref:`xml_topicData`
    -   :ref:`xml_userData`

*   :ref:`historymemorypoliciesXML`
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

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<port>``
    - RTPS port number of the locator. |br|
      *Physical port* in UDP, *logical port* in TCP.
    - ``uint16_t``
    - 0
  * - ``<physical_port>``
    - TCP's *physical port*.
    - ``uint16_t``
    - 0
  * - ``<address>``
    - IP address of the locator.
    - ``string`` (IPv4/IPv6 format |br|
      or DNS name)
    - Empty
  * - ``<unique_lan_id>``
    - The LAN ID uniquely identifies the LAN the |br|
      locator belongs to (**TCPv4 only**).
    - ``string`` (16 bytes)
    - Empty
  * - ``<wan_address>``
    - WAN IPv4 address (**TCPv4 only**).
    - ``string`` (IPv4 format)
    - ``0.0.0.0``

**Example**

The following example shows the implementation of one locator of each transport protocol in
``<defaultUnicastLocatorList>``.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-LOCATOR-LIST<-->
    :end-before: <!--><-->

.. _externalLocatorListType:

ExternalLocatorListType
^^^^^^^^^^^^^^^^^^^^^^^

It represents a list of external locator entries.
Each entry can be a ``<udpv4>`` or a ``<udpv6>`` tag.
These tags can be configured with the following attributes:

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``externality``
    - Number of hops from the participant's host to the |br|
      LAN represented by the external locator. |br|
      Valid values: from 1 to 255.
    - ``uint8_t``
    - 1
  * - ``cost``
    - Communication cost relative to other locators on |br|
      the same externality level. |br|
      Valid values: from 0 to 255.
    - ``uint8_t``
    - 0
  * - ``mask``
    - Number of significant bits on the LAN represented |br|
      by the external locator. |br|
      Valid values: from 1 to 31 (UDPv4) or 127 (UDPv6)
    - ``uint8_t``
    - 24

They should contain the following tags:

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
  * - ``<port>``
    - UDP port number of the locator. |br|
      The UDP port number should be valid.
    - ``uint16_t``
  * - ``<address>``
    - IP address of the locator.
    - ``string`` (IPv4/IPv6 format |br|
      or DNS name)

**Example**

The following example shows the implementation of one locator of each transport protocol in
``<default_external_unicast_locators>``.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-EXTERNAL-LOCATOR-LIST<-->
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
A DurationType is defined by at least one mandatory element of two possible ones: ``<sec>`` plus ``<nanosec>``.
An infinite value can be specified by using the values :cpp:concept:`DURATION_INFINITY`,
:cpp:concept:`DURATION_INFINITE_SEC` and :cpp:concept:`DURATION_INFINITE_NSEC`.

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<sec>``
    - Number of seconds.
    - ``int32_t``
    - 0
  * - ``<nanosec>``
    - Number of nanoseconds.
    - ``uint32_t``
    - 0

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-DURATION<-->
    :end-before: <!--><-->

.. _TopicType:

TopicType
^^^^^^^^^

This XML element allows the configuration of the specific :ref:`historyqospolicy` and :ref:`resourcelimitsqospolicy`
QoS of the Datawriters and DataReaders in which this element is defined inside of.
Also, it sets the :ref:`dds_layer_topic_topicQos` configuration with the policies detailed.

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
  * - ``<historyQos>``
    - It controls the behavior of *Fast DDS* |br|
      when the value of an instance changes  |br|
      before it is finally communicated to |br|
      some of its existing DataReaders. |br|
    - :ref:`hQos`
  * - ``<resourceLimitsQos>``
    - It controls the resources that *Fast DDS* |br|
      can use in order to meet the |br|
      requirements imposed by the application |br|
      and other QoS settings.
    - :ref:`rLsQos`

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
| ``<max_samples>``              | It must verify that:                                      | ``int32_t``   | 5000    |
|                                | ``<max_samples>`` `>=` ``<max_samples_per_instance>``.    |               |         |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<max_instances>``            | It defines the maximum number of instances.               | ``int32_t``   | 10      |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<max_samples_per_instance>`` | It must verify that: :ref:`HistoryQos <hQos>`             | ``int32_t``   | 400     |
|                                | ``<depth>`` `<=` ``<max_samples_per_instance>``.          |               |         |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<allocated_samples>``        | It controls the maximum number of samples to be stored.   | ``int32_t``   | 100     |
+--------------------------------+-----------------------------------------------------------+---------------+---------+
| ``<extra_samples>``            | The number of extra samples to allocate on the pool.      | ``int32_t``   | 1       |
+--------------------------------+-----------------------------------------------------------+---------------+---------+

.. _CommonQOS:

QoS
^^^

The Quality of Service (QoS) is used to specify the behavior of the Service, allowing the user to define how each
|Entity| will behave.
Please refer to the :ref:`dds_layer_core_policy` section for more information on QoS.

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
  * - ``<data_sharing>``
    - See :ref:`datasharingqospolicy`
    - :ref:`xml_datasharing`
  * - ``<deadline>``
    - See :ref:`deadlineqospolicy`.
    - :ref:`xml_deadline`
  * - ``<disable_heartbeat_piggyback>``
    - See :ref:`disableheartbeatpiggyback`.
    - :ref:`xml_disableheartbeatpiggyback`
  * - ``<disablePositiveAcks>``
    - See :ref:`disablepositiveacksqospolicy`.
    - :ref:`xml_disablepositiveacks`
  * - ``<durability>``
    - See :ref:`durabilityqospolicy`.
    - :ref:`xml_durability`
  * - ``<groupData>``
    - See :ref:`groupqospolicy`.
    - :ref:`xml_groupData`
  * - ``<latencyBudget>``
    - See :ref:`latencybudgetqospolicy`.
    - :ref:`xml_latencybudget`
  * - ``<lifespan>``
    - See :ref:`lifespanqospolicy`.
    - :ref:`xml_lifespan`
  * - ``<liveliness>``
    - See :ref:`livelinessqospolicy`.
    - :ref:`xml_liveliness`
  * - ``<ownership>``
    - See :ref:`ownershipqospolicy`.
    - :ref:`xml_ownership`
  * - ``<ownershipStrength>``
    - See :ref:`ownershipstrengthqospolicy`.
    - :ref:`xml_ownershipstrength`
  * - ``<partition>``
    - See :ref:`partitionqospolicy`.
    - :ref:`xml_partition`
  * - ``<publishMode>``
    - See :ref:`publishmodeqospolicy`.
    - :ref:`xml_publishmode`
  * - ``<reliability>``
    - See :ref:`reliabilityqospolicy`.
    - :ref:`xml_reliability`
  * - ``<topicData>``
    - See :ref:`topicdataqospolicy`.
    - :ref:`xml_topicData`
  * - ``<userData>``
    - See :ref:`userdataqospolicy`.
    - :ref:`xml_userData`

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-QOS<-->
    :end-before: <!--><-->

.. _xml_datasharing:

Data-Sharing
""""""""""""

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<kind>``
     - See :ref:`datasharingkind`
     - |DATASHARING_AUTO-xml-api| |br|
       |DATASHARING_ON-api| |br|
       |DATASHARING_OFF-api| |br|
     - |DATASHARING_AUTO-xml-api|
   * - ``<shared_dir>``
     - Directory used for the memory-mapped files.
     - ``string``
     - Empty
   * - ``<max_domains>``
     - Maximum number of Data-Sharing domain IDs |br|
       in the local or remote endpoints.
     - ``uint32_t``
     - 0 (unlimited)
   * - ``<domain_ids>``
     - List of Data-Sharing domain IDs configured |br|
       for the current endpoint.
     - ``<domainId>``
     - Empty list

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
   * - ``<domainId>``
     - Domain ID to be used by the endpoint for Data-Sharing.
     - ``uint32_t``

.. _xml_deadline:

Deadline
""""""""
+---------------------------+----------------------------------+---------------------+---------------------------------+
| Name                      | Description                      | Values              | Default                         |
+===========================+==================================+=====================+=================================+
| ``<period>``              | See :ref:`deadlineqospolicy`.    | :ref:`DurationType` | |c_TimeInfinite-api|            |
+---------------------------+----------------------------------+---------------------+---------------------------------+

.. _xml_disableheartbeatpiggyback:

DisableHeartbeatPiggyback
"""""""""""""""""""""""""

+----------------------------------------------+---------------------------------------+----------+-----------+
| Name                                         | Description                           | Values   | Default   |
+==============================================+=======================================+==========+===========+
| ``<disable_heartbeat_piggyback>``            | See :ref:`disableheartbeatpiggyback`. | ``bool`` | ``false`` |
+----------------------------------------------+---------------------------------------+----------+-----------+

.. important::

    This configuration is only available for :ref:`DataWriter QoS profile configuration <publisherprofiles>`.

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

.. _xml_durability:

Durability
""""""""""

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<kind>``
    - See :ref:`durabilitykind`.
    - |VOLATILE-xml-api| |br|
      |TRANSIENT_LOCAL-xml-api| |br|
      |TRANSIENT-xml-api| |br|
      |PERSISTENT-xml-api|
    - DataReaders: |VOLATILE-xml-api| |br|
      DataWriters: |TRANSIENT_LOCAL-xml-api| |br|

.. _xml_groupData:

GroupData
"""""""""

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<value>``
    - See :ref:`groupqospolicy`.
    - ``string`` (std::vector<|octet-api|>)
    - Empty

.. _xml_latencybudget:

LatencyBudget
"""""""""""""

+---------------------------+-------------------------------------------------+---------------------+------------------+
| Name                      | Description                                     | Values              | Default          |
+===========================+=================================================+=====================+==================+
| ``<duration>``            | See :ref:`latencybudgetqospolicy`.              | :ref:`DurationType` | 0                |
+---------------------------+-------------------------------------------------+---------------------+------------------+

.. _xml_lifespan:

Lifespan
""""""""

+---------------------------+----------------------------------+---------------------+---------------------------------+
| Name                      | Description                      | Values              | Default                         |
+===========================+==================================+=====================+=================================+
| ``<duration>``            | See :ref:`lifespanqospolicy`.    | :ref:`DurationType` | |c_TimeInfinite-api|            |
+---------------------------+----------------------------------+---------------------+---------------------------------+

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
| ``<announcement_period>`` | See :ref:`livelinessqospolicy`. | :ref:`DurationType`             | |c_TimeInfinite-api| |
+---------------------------+---------------------------------+---------------------------------+----------------------+

.. _xml_ownership:

Ownership
"""""""""

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<kind>``
    - See :ref:`ownershipqospolicykind`.
    - |SHARED-xml-api| |br|
      |EXCLUSIVE-xml-api|
    - |SHARED-xml-api|

.. _xml_ownershipstrength:

Ownership Strength
""""""""""""""""""

+------------------+----------------------------------------+--------------------------------------+-------------------+
| Name             | Description                            | Values                               | Default           |
+==================+========================================+======================================+===================+
| ``<value>``      | See                                    | ``uint32_t``                         | 0                 |
|                  | :ref:`ownershipstrengthqospolicy`.     |                                      |                   |
+------------------+----------------------------------------+--------------------------------------+-------------------+

.. important::

    This configuration is only available for :ref:`DataWriter QoS profile configuration <publisherprofiles>`.

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

.. _xml_publishmode:

PublishMode
"""""""""""

+--------------------------+---------------------------------------+------------------+------------------+
| Name                     | Description                           | Values           | Default          |
+==========================+=======================================+==================+==================+
| ``<kind>``               | See :ref:`publishmodeqospolicy`.      | ``ASYNCHRONOUS`` | ``ASYNCHRONOUS`` |
|                          |                                       +------------------+                  |
|                          |                                       | ``SYNCHRONOUS``  |                  |
+--------------------------+---------------------------------------+------------------+------------------+
|``<flow_controller_name>``| :ref:`flowcontrollersqos` name.       | ``<string>``     | Empty            |
+--------------------------+---------------------------------------+------------------+------------------+

.. important::

    This configuration is only available for :ref:`DataWriter QoS profile configuration <publisherprofiles>`.

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

.. _xml_topicData:

TopicData
"""""""""

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<value>``
    - See :ref:`topicdataqospolicy`.
    - ``string`` (std::vector<|octet-api|>)
    - Empty

.. _xml_userData:

UserData
""""""""

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<value>``
    - See :ref:`userdataqospolicy`.
    - ``string`` (std::vector<|octet-api|>)
    - Empty

.. _historymemorypoliciesXML:

HistoryMemoryPolicy
^^^^^^^^^^^^^^^^^^^

Indicates the way the memory is managed in terms of dealing with the CacheChanges of the :ref:`rtpsendpointqos`.

.. list-table::
  :header-rows: 1
  :align: left

  * - Name
    - Description
    - Values
    - Default
  * - ``<historyMemoryPolicy>``
    - Four different options as described |br|
      in :ref:`memorymanagementpolicy`.
    - |PREALLOCATED-xml-api| |br|
      |PREALLOCATED_WITH_REALLOC-xml-api| |br|
      |DYNAMIC-xml-api| |br|
      |DYNAMIC_REUSABLE-xml-api|
    - |PREALLOCATED-xml-api|

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

.. _flowcontrollers_xml:

Flow Controller Descriptors
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This ``<flow_controller_descriptor_list>`` element configures the list of flow controllers of a participant,
so they can later be used on its DataWriters.
Please refer to :ref:`flowcontrollersqos` for a detailed documentation.

.. list-table::
   :header-rows: 1
   :align: left

   * - Data Member Name
     - Type
     - Default Value
   * - ``<name>``
     - ``string``
     - Empty
   * - ``<scheduler>``
     - |FlowControllerSchedulerPolicy-api|
     - |FIFO_SCHED_POLICY-api|
   * - ``<max_bytes_per_period>``
     - ``int32_t``
     - 0 (i.e. infinite)
   * - ``<period_ms>``
     - ``uint64_t``
     - 100
