.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. role:: raw-html(raw)
    :format: html


.. _eprosima_extensions:

eProsima Extensions
-------------------

The eProsima QoS Policies extensions are those that allow changing the values of the RTPS layer configurable settings.

.. contents::
    :local:
    :backlinks: none
    :depth: 1


.. _datasharingqospolicy:

DataSharingQosPolicy
^^^^^^^^^^^^^^^^^^^^

This additional QoS allows configuring the data-sharing delivery communication between a writer and a reader.
Please, see :ref:`datasharing-delivery` for a description of the data-sharing delivery functionality.

List of QoS Policy data members:

+-------------------------+------------------------+-------------------------------------------+---------------+
| Data Member             | Type                   | Accessor                                  | Default Value |
+=========================+========================+===========================================+===============+
| Data-sharing kind       | :ref:`datasharingkind` | |DataSharingQosPolicy::kind-api|          | ``AUTO``      |
+-------------------------+------------------------+-------------------------------------------+---------------+
| Shared memory directory | ``string``             | |DataSharingQosPolicy::shm_directory-api| | Empty string  |
+-------------------------+------------------------+-------------------------------------------+---------------+
| Maximum domain number   | ``uint32_t``           | |DataSharingQosPolicy::max_domains-api|   | 0 (unlimited) |
+-------------------------+------------------------+-------------------------------------------+---------------+
| Data-sharing domain IDs | ``vector<uint64_t>``   | |DataSharingQosPolicy::domain_ids-api|    | Empty         |
+-------------------------+------------------------+-------------------------------------------+---------------+

* Data-sharing kind:
  Specifies the behavior of data-sharing delivery.
  See :ref:`DataSharingKind` for a description of possible values and their effect.
* Shared memory directory:
  The directory that will be used for the memory-mapped files.
  If none is configured, then the system default directory will be used.
* Maximum domain number:
  Establishes the maximum number of data-sharing domain IDs in the local or remote endpoints.
  Domain IDs are exchanged between data-sharing delivery compatible endpoints.
  If this value is lower that the size of the list for any remote endpoint, the matching may fail.
  A value of zero represents unlimited number of IDs.
* Data sharing domain IDs:
  The list of data-sharing domain IDs configured for the current |DataWriter| or |DataReader|.
  If no ID is provided, the system will create a unique one for the current machine.

.. note::
     This QoS Policy concerns to |DataWriter| and |DataReader| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _datasharingkind:

DataSharingKind
"""""""""""""""

There are three possible values (see |DataSharingKind-api|):

* |DATASHARING_OFF-api|:
  The data-sharing delivery is disabled.
  No communication will be performed using data-sharing delivery functionality.
* |DATASHARING_ON-api|:
  The data-sharing delivery is manually enabled.
  An error will occur if the current topic is not :ref:`compatible<datasharing-delivery-constraints>`
  with data-sharing delivery.
  Communication with remote entities that share at least one data-sharing domain ID
  will be done using data-sharing delivery functionality.
* |DATASHARING_AUTO-api|:
  data-sharing delivery will be activated if the current topic is
  :ref:`compatible<datasharing-delivery-constraints>` with data-sharing,
  and deactivated if not.


.. datasharingconfiguration:

Data-sharing configuration helper functions
"""""""""""""""""""""""""""""""""""""""""""

In order to set the data-sharing delivery configuration, one of the following helper member functions must be used.
There is one for each :ref:`datasharingkind` flavor:

+----------------------------------+---------------------------+-------------------------+-------------------------+
| Function                         | Resulting DataSharingKind | Shared memory directory | Data sharing domain IDs |
+==================================+===========================+=========================+=========================+
| |DataSharingQosPolicy::auto-api| | |DATASHARING_AUTO-api|    | Optional                | Optional                |
+----------------------------------+---------------------------+-------------------------+-------------------------+
| |DataSharingQosPolicy::on-api|   | |DATASHARING_ON-api|      | Mandatory               | Optional                |
+----------------------------------+---------------------------+-------------------------+-------------------------+
| |DataSharingQosPolicy::off-api|  | |DATASHARING_OFF-api|     | N/A                     | N/A                     |
+----------------------------------+---------------------------+-------------------------+-------------------------+

Instead of defining the data-sharing domain IDs on these helper functions,
you can add them later with the |DataSharingQosPolicy::add_domain_id-api| function.
Beware that adding a new domain ID counts as modifying the QosPolicy,
so it must be done before the entity is enabled.


Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_DATASHARING_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-QOS-DATASHARING<-->
    :end-before: <!--><-->

.. _disablepositiveacksqospolicy:

DisablePositiveACKsQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This additional QoS allows reducing network traffic when strict reliable communication is not required and bandwidth is
limited.
It consists in changing the default behavior by which positive acks are sent from readers to writers.
Instead, only negative acks will be sent when a reader is missing a sample, but writers will keep data for an adjustable
time before considering it as acknowledged.
See |DisablePositiveACKsQosPolicy-api|.

List of QoS Policy data members:

+-------------------------------------------------------------+------------------------------+-------------------------+
| Data Member Name                                            | Type                         | Default Value           |
+=============================================================+==============================+=========================+
| |DisablePositiveACKsQosPolicy::enabled-api|                 | ``bool``                     | ``false``               |
+-------------------------------------------------------------+------------------------------+-------------------------+
| |DisablePositiveACKsQosPolicy::duration-api|                | |Duration_t-api|             | |c_TimeInfinite-api|    |
+-------------------------------------------------------------+------------------------------+-------------------------+

* |DisablePositiveACKsQosPolicy::enabled-api|:
  Specifies if the QoS is enabled or not. If it is true means that the positive acks are disabled and the
  DataReader only sends negative acks. Otherwise, both positive and negative acks are sent.
* |DisablePositiveACKsQosPolicy::duration-api|:
  State the duration that the DataWriters keep the data before considering it as acknowledged.
  This value does not apply to DataReaders.

.. note::
     This QoS Policy concerns to |DataWriter| and |DataReader| entities.
     :raw-html:`<br />`
     The |DisablePositiveACKsQosPolicy::enabled-api| Data Member cannot be modified on enabled entities.
     Thus, this feature must be set up during initialization.
     Only the |DisablePositiveACKsQosPolicy::duration-api| Data Member can be modified at runtime.
.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule.
    See :ref:`disableacks_compatibilityrule` for further details.

.. _disableacks_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between DisablePositiveACKsQosPolicy in DataReaders and DataWriters, the DataReader
cannot have this QoS enabled if the DataWriter have it disabled.

Table with the possible combinations:

+----------------------------+----------------------------+-----------------+
| DataWriter `enabled` value | DataReader `enabled` value | Compatibility   |
+============================+============================+=================+
| ``true``                   | ``true``                   | Yes             |
+----------------------------+----------------------------+-----------------+
| ``true``                   | ``false``                  | Yes             |
+----------------------------+----------------------------+-----------------+
| ``false``                  | ``true``                   | No              |
+----------------------------+----------------------------+-----------------+
| ``false``                  | ``false``                  | Yes             |
+----------------------------+----------------------------+-----------------+

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_DISABLE_POSITIVE_ACKS_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->PUBSUB_API_CONF_PUBSUB_DISABLE_POSITIVE_ACKS
    :end-before: <!--><-->


.. _flowcontrollersqos:

FlowControllersQos
^^^^^^^^^^^^^^^^^^

This QoS configures the list of flow controllers of a participant, so they can later be used on
its DataWriters.
It is a vector of shared pointers to |FlowControllerDescriptor-api|, which has the following fields:

+------------------------------------------------------+-------------------------------------+-------------------------+
| Data Member Name                                     | Type                                | Default Value           |
+======================================================+=====================================+=========================+
| |FlowControllerDescriptor::name-api|                 | ``const char *``                    |                         |
+------------------------------------------------------+-------------------------------------+-------------------------+
| |FlowControllerDescriptor::scheduler-api|            | |FlowControllerSchedulerPolicy-api| | |FIFO_SCHED_POLICY-api| |
+------------------------------------------------------+-------------------------------------+-------------------------+
| |FlowControllerDescriptor::max_bytes_per_period-api| | ``int32_t``                         | 0 (i.e. infinite)       |
+------------------------------------------------------+-------------------------------------+-------------------------+
| |FlowControllerDescriptor::period_ms-api|            | ``uint64_t``                        | 100                     |
+------------------------------------------------------+-------------------------------------+-------------------------+

Please refer to :ref:`flow-controllers` section for more information.


.. note::
     This QoS Policy concerns to |DomainParticipant| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.


.. _participantresourcelimitsqos:

ParticipantResourceLimitsQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This QoS configures allocation limits and the use of physical memory for internal resources.
See |ParticipantResourceLimitsQos-api|.

List of QoS Policy data members:

+--------------------------------------------------------------------------+-------------------------------------------+
| Data Member Name                                                         | Type                                      |
+==========================================================================+===========================================+
| |ParticipantResourceLimitsQos::locators-api|                             | :ref:`remotelocatorsallocationattributes` |
+--------------------------------------------------------------------------+-------------------------------------------+
| |ParticipantResourceLimitsQos::participants-api|                         | :ref:`resourcelimitedcontainerconfig`     |
+--------------------------------------------------------------------------+-------------------------------------------+
| |ParticipantResourceLimitsQos::readers-api|                              | :ref:`resourcelimitedcontainerconfig`     |
+--------------------------------------------------------------------------+-------------------------------------------+
| |ParticipantResourceLimitsQos::writers-api|                              | :ref:`resourcelimitedcontainerconfig`     |
+--------------------------------------------------------------------------+-------------------------------------------+
| |ParticipantResourceLimitsQos::send_buffers-api|                         | :ref:`sendbuffersallocationattributes`    |
+--------------------------------------------------------------------------+-------------------------------------------+
| |ParticipantResourceLimitsQos::data_limits-api|                          | :ref:`variablelengthdatalimits`           |
+--------------------------------------------------------------------------+-------------------------------------------+
| |ParticipantResourceLimitsQos::content_filter-api|                       | :ref:`contentfilterlimits`                |
+--------------------------------------------------------------------------+-------------------------------------------+

* |ParticipantResourceLimitsQos::locators-api|:
  Defines the limits for collections of remote locators.
* |ParticipantResourceLimitsQos::participants-api|:
  Specifies the allocation behavior and limits for collections dependent on the total number of
  participants.
* |ParticipantResourceLimitsQos::readers-api|:
  Specifies the allocation behavior and limits for collections dependent on the total number of
  readers per participant.
* |ParticipantResourceLimitsQos::writers-api|:
  Specifies the allocation behavior and limits for collections dependent on the total number of
  writers per participant.
* |ParticipantResourceLimitsQos::send_buffers-api|:
  Defines the allocation behavior and limits for the send buffer manager.
* |ParticipantResourceLimitsQos::data_limits-api|:
  States the limits for variable-length data.
* |ParticipantResourceLimitsQos::content_filter-api|:
  States the limits for content-filter discovery information.

.. note::
     This QoS Policy concerns to |DomainParticipant| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _remotelocatorsallocationattributes:

RemoteLocatorsAllocationAttributes
""""""""""""""""""""""""""""""""""

This structure holds the limits for the remote locators' collections.
See |RemoteLocatorsAllocationAttributes-api|.

List of structure members:

+-----------------------------------------------------------------------------------------+------------+---------------+
| Member Name                                                                             | Type       | Default Value |
+=========================================================================================+============+===============+
| |RemoteLocatorsAllocationAttributes::max_unicast_locators-api|                          | ``size_t`` | 4             |
+-----------------------------------------------------------------------------------------+------------+---------------+
| |RemoteLocatorsAllocationAttributes::max_multicast_locators-api|                        | ``size_t`` | 1             |
+-----------------------------------------------------------------------------------------+------------+---------------+

* |RemoteLocatorsAllocationAttributes::max_unicast_locators-api|:
  This member controls the maximum number of unicast locators to keep for each discovered
  remote entity.
  It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain.
* |RemoteLocatorsAllocationAttributes::max_multicast_locators-api|:
  This member controls the maximum number of multicast locators to keep for each discovered
  remote entity.
  The default value is usually enough, as it does not make sense to add more than one multicast locator per entity.


.. _resourcelimitedcontainerconfig:

ResourceLimitedContainerConfig
""""""""""""""""""""""""""""""

This structure holds the limits of a resource limited collection, as well as the allocation configuration, which can be
fixed size or dynamic size.

List of structure members:

+-------------------------------------------------+------------+-------------------------------------------------------+
| Member Name                                     | Type       | Default Value                                         |
+=================================================+============+=======================================================+
| |ResourceLimitedContainerConfig::initial-api|   | ``size_t`` | 0                                                     |
+-------------------------------------------------+------------+-------------------------------------------------------+
| |ResourceLimitedContainerConfig::maximum-api|   | ``size_t`` | :func:`std::numeric_limits<size_t>::max()`            |
+-------------------------------------------------+------------+-------------------------------------------------------+
| |ResourceLimitedContainerConfig::increment-api| | ``size_t`` | 1 (dynamic size), 0 (fixed size)                      |
+-------------------------------------------------+------------+-------------------------------------------------------+

* |ResourceLimitedContainerConfig::initial-api|:
  Indicates the number of elements to preallocate in the collection.
* |ResourceLimitedContainerConfig::maximum-api|:
  Specifies the maximum number of elements allowed in the collection.
* |ResourceLimitedContainerConfig::increment-api|:
  States the number of items to add when the reserved capacity limit is reached. This member has a
  different default value depending on the allocation configuration chosen.

.. _sendbuffersallocationattributes:

SendBuffersAllocationAttributes
"""""""""""""""""""""""""""""""

This structure holds the limits for the allocations of the send buffers.
See |SendBuffersAllocationAttributes-api|.

List of structure members:

+------------------------------------------------------------------------+-------------------------+-------------------+
| Member Name                                                            | Type                    | Default Value     |
+========================================================================+=========================+===================+
| |SendBuffersAllocationAttributes::preallocated_number-api|             | ``size_t``              | 0                 |
+------------------------------------------------------------------------+-------------------------+-------------------+
| |SendBuffersAllocationAttributes::dynamic-api|                         | ``bool``                | ``false``         |
+------------------------------------------------------------------------+-------------------------+-------------------+

* |SendBuffersAllocationAttributes::preallocated_number-api|:
  This member controls the initial number of send buffers to be allocated.
  The default value will perform an initial guess of the number of buffers required, based on the number of threads
  from which a send operation could be started.
* |SendBuffersAllocationAttributes::dynamic-api|:
  This member controls how the buffer manager behaves when a send buffer is not available.
  When true, a new buffer will be created. Otherwise, it will wait for a buffer to be returned.

.. _variablelengthdatalimits:

VariableLengthDataLimits
""""""""""""""""""""""""

This structure holds the limits for variable-length data.
See |VariableLengthDataLimits-api|.

List of structure members:

+----------------------------------------------------------------------------------+---------------+-------------------+
| Member Name                                                                      | Type          | Default Value     |
+==================================================================================+===============+===================+
| |VariableLengthDataLimits::max_properties-api|                                   | ``size_t``    | 0                 |
+----------------------------------------------------------------------------------+---------------+-------------------+
| |VariableLengthDataLimits::max_user_data-api|                                    | ``size_t``    | 0                 |
+----------------------------------------------------------------------------------+---------------+-------------------+
| |VariableLengthDataLimits::max_partitions-api|                                   | ``size_t``    | 0                 |
+----------------------------------------------------------------------------------+---------------+-------------------+

* |VariableLengthDataLimits::max_properties-api|:
  Defines the maximum size, in octets, of the properties data in the local or remote participant.
* |VariableLengthDataLimits::max_user_data-api|:
  Establishes the maximum size, in octets, of the user data in the local or remote participant.
* |VariableLengthDataLimits::max_partitions-api|:
  States the maximum size, in octets, of the partitions data in the local or remote participant.

.. _contentfilterlimits:

ContentFilterProperty::AllocationConfiguration
""""""""""""""""""""""""""""""""""""""""""""""

This structure holds the limits for content-filter related discovery information.
See |ContentFilterProperty::AllocationConfiguration-api|.

List of structure members:

.. list-table::
   :header-rows: 1
   :align: left

   * - Member Name
     - Type
     - Default Value
   * - |ContentFilterProperty::AllocationConfiguration::expression_initial_size-api|
     - ``size_t``
     - 0
   * - |ContentFilterProperty::AllocationConfiguration::expression_parameters-api|
     - :ref:`resourcelimitedcontainerconfig`
     - ``{0, 100, 1}``

* |ContentFilterProperty::AllocationConfiguration::expression_initial_size-api|:
  Preallocated size of the filter expression.
* |ContentFilterProperty::AllocationConfiguration::expression_parameters-api|:
  Allocation configuration for the list of expression parameters.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_PARTICIPANT_RESOURCE_LIMITS_QOS_POLICY
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-ALLOCATION-QOS-EXAMPLE
    :end-before: <data_writer

.. _propertypolicyqos:

PropertyPolicyQos
^^^^^^^^^^^^^^^^^

This additional QoS Policy (|PropertyPolicyQos-api|) stores name/value pairs that can be used to configure certain
DDS settings that cannot be configured directly using an standard QoS Policy.
For the complete list of settings that can be configured with this QoS Policy, please refer to :ref:`property_policies`.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_PROPERTY_POLICY_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_PROPERTY_POLICY
    :end-before: <!--><-->

.. _publishmodeqospolicy:

PublishModeQosPolicy
^^^^^^^^^^^^^^^^^^^^

This QoS Policy configures how the |DataWriter| sends the data.
See |PublishModeQosPolicy-api|.

It also configures the name of the flow controller to use when asynchronous publishing is used.
It should be the name of a flow controller registered on the creation of the DomainParticipant.
See |FlowControllersQos|.

List of QoS Policy data members:

.. list-table::
   :header-rows: 1
   :align: left

   * - Data Member Name
     - Type
     - Default Value
   * - |PublishModeQosPolicy::kind-api|
     - :ref:`publishmodeqospolicykind`
     - |SYNCHRONOUS_PUBLISH_MODE-api|
   * - |PublishModeQosPolicy::flow_ctrl_name-api|
     - ``const char *``
     - |FASTDDS_FLOW_CONTROLLER_DEFAULT-api|

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _publishmodeqospolicykind:

PublishModeQosPolicyKind
""""""""""""""""""""""""

There are two possible values (see |PublishModeQosPolicyKind-api|):

* |SYNCHRONOUS_PUBLISH_MODE-api|: The data is sent in the context of the user thread that calls the write operation.
* |ASYNCHRONOUS_PUBLISH_MODE-api|: An internal thread takes the responsibility of sending the data asynchronously.
  The write operation returns before the data is actually sent.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_PUBLISH_MODE_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-QOS-PUBLISHMODE<-->
    :end-before: <!--><-->

.. _readerresourcelimitsqos:

ReaderResourceLimitsQos
^^^^^^^^^^^^^^^^^^^^^^^

This QoS Policy states the limits for the matched |DataWriters|' resource limited collections based on the maximum
number of DataWriters that are going to match with the |DataReader|.
See |ReaderResourceLimitsQos-api|.

List of QoS Policy data members:

+--------------------------------------------------------------------------+-------------------------------------------+
| Data Member Name                                                         | Type                                      |
+==========================================================================+===========================================+
| |ReaderResourceLimitsQos::matched_publisher_allocation-api|              | :ref:`resourcelimitedcontainerconfig`     |
+--------------------------------------------------------------------------+-------------------------------------------+


.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_READER_RESOURCE_LIMITS_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_READER_RESOURCE_LIMITS_QOS<-->
    :end-before: <!--><-->

.. _rtpsendpointqos:

RTPSEndpointQos
^^^^^^^^^^^^^^^

This QoS Policy configures the aspects of an RTPS endpoint, such as the list of locators, the identifiers, and the
history memory policy.
See |RTPSEndpointQos-api|.

List of QoS Policy data members:

+-----------------------------------------------+-------------------------------+--------------------------------------+
| Data Member Name                              | Type                          | Default Value                        |
+===============================================+===============================+======================================+
| |RTPSEndpointQos::unicast_locator_list-api|   | |LocatorList_t-api|           | Empty List                           |
+-----------------------------------------------+-------------------------------+--------------------------------------+
| |RTPSEndpointQos::multicast_locator_list-api| | |LocatorList_t-api|           | Empty List                           |
+-----------------------------------------------+-------------------------------+--------------------------------------+
| |RTPSEndpointQos::remote_locator_list-api|    | |LocatorList_t-api|           | Empty List                           |
+-----------------------------------------------+-------------------------------+--------------------------------------+
| |RTPSEndpointQos::user_defined_id-api|        | ``int16_t``                   | -1                                   |
+-----------------------------------------------+-------------------------------+--------------------------------------+
| |RTPSEndpointQos::entity_id-api|              | ``int16_t``                   | -1                                   |
+-----------------------------------------------+-------------------------------+--------------------------------------+
| |RTPSEndpointQos::history_memory_policy-api|  | :ref:`memorymanagementpolicy` | |PREALLOCATED_MEMORY_MODE-api|       |
+-----------------------------------------------+-------------------------------+--------------------------------------+

* |RTPSEndpointQos::unicast_locator_list-api|:
  Defines the list of unicast locators associated to the DDS Entity.
  DataReaders and DataWriters inherit the list of unicast locators set in the DomainParticipant, but it can be
  changed by means of this QoS.
* |RTPSEndpointQos::multicast_locator_list-api|:
  Stores the list of multicast locators associated to the DDS Entity.
  By default, DataReaders and DataWriters do not use any multicast locator, but it can be changed by means of this QoS.
* |RTPSEndpointQos::remote_locator_list-api|:
  States the list of remote locators associated to the DDS Entity.
* |RTPSEndpointQos::user_defined_id-api|:
  Establishes the unique identifier used for StaticEndpointDiscovery.
* |RTPSEndpointQos::entity_id-api|:
  The user can specify the identifier for the endpoint.
* |RTPSEndpointQos::history_memory_policy-api|:
  Indicates the way the memory is managed in terms of dealing with the CacheChanges.

.. note::
     This QoS Policy concerns to |DataWriter| and |DataReader| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _memorymanagementpolicy:

MemoryManagementPolicy
""""""""""""""""""""""

There are four possible values (see |MemoryManagementPolicy-api|):

* |PREALLOCATED_MEMORY_MODE-api|:
  This option sets the size to the maximum of each data type. It produces the largest
  memory footprint but the smallest allocation count.
* |PREALLOCATED_WITH_REALLOC_MEMORY_MODE-api|:
  This option set the size to the default for each data type and it requires
  reallocation when a bigger message arrives. It produces a lower memory footprint at the expense of increasing the
  allocation count.
* |DYNAMIC_RESERVE_MEMORY_MODE-api|:
  This option allocates the size dynamically at the time of message arrival. It
  produces the least memory footprint but the highest allocation count.
* |DYNAMIC_REUSABLE_MEMORY_MODE-api|:
  This option is similar to ``DYNAMIC_RESERVE_MEMORY_MODE``, but the allocated memory
  is reused for future messages.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RTPS_ENDPOINT_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RTPS_ENDPOINT_QOS<-->
    :end-before: <!--><-->

.. _rtpsreliablereaderqos:

RTPSReliableReaderQos
^^^^^^^^^^^^^^^^^^^^^

This RTPS QoS Policy allows the configuration of several RTPS reliable reader's aspects.
See |RTPSReliableReaderQos-api|.

List of QoS Policy data members:

+--------------------------------------------------------------------------------+-------------------------------------+
| Data Member Name                                                               | Type                                |
+================================================================================+=====================================+
| |RTPSReliableReaderQos::times-api|                                             | :ref:`readertimes`                  |
+--------------------------------------------------------------------------------+-------------------------------------+
| |RTPSReliableReaderQos::disable_positive_ACKs-api|                             | :ref:`disablepositiveacksqospolicy` |
+--------------------------------------------------------------------------------+-------------------------------------+

* |RTPSReliableReaderQos::times-api|:
  Defines the duration of the RTPSReader events. See :ref:`readertimes` for further details.
* |RTPSReliableReaderQos::disable_positive_ACKs-api|:
  Configures the settings to disable the positive acks.
  See :ref:`disablepositiveacksqospolicy` for further details.

.. note::
     This QoS Policy concerns to |DataReader| entities.
     :raw-html:`<br />`
     Only the |DisablePositiveACKsQosPolicy::duration-api| Data Member of the :ref:`disablepositiveacksqospolicy` and the |RTPSReliableReaderQos::times-api|
     Data Member can be modified on enabled entities.

.. _readertimes:

ReaderTimes
"""""""""""

This structure defines the times associated with the Reliable Readers' events.
See |ReaderTimes-api|.

List of structure members:

+-----------------------------------------------------------------------------------+------------------+---------------+
| Member Name                                                                       | Type             | Default Value |
+===================================================================================+==================+===============+
| |ReaderTimes::initialAcknackDelay-api|                                            | |Duration_t-api| | 70 ms         |
+-----------------------------------------------------------------------------------+------------------+---------------+
| |ReaderTimes::heartbeatResponseDelay-api|                                         | |Duration_t-api| | 5 ms          |
+-----------------------------------------------------------------------------------+------------------+---------------+

* |ReaderTimes::initialAcknackDelay-api|:
  Defines the duration of the initial acknack delay.
* |ReaderTimes::heartbeatResponseDelay-api|:
  Establishes the duration of the delay applied when a heartbeat message is received.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RTPS_RELIABLE_READER_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RTPS_RELIABLE_READER_QOS<-->
    :end-before: <!--><-->


.. _rtpsreliablewriterqos:

RTPSReliableWriterQos
^^^^^^^^^^^^^^^^^^^^^

This RTPS QoS Policy allows the configuration of several RTPS reliable writer's aspects.
See |RTPSReliableWriterQos-api|.

List of QoS Policy data members:

+--------------------------------------------------------------------------------+-------------------------------------+
| Data Member Name                                                               | Type                                |
+================================================================================+=====================================+
| |RTPSReliableWriterQos::times-api|                                             | :ref:`writertimes`                  |
+--------------------------------------------------------------------------------+-------------------------------------+
| |RTPSReliableWriterQos::disable_positive_acks-api|                             | :ref:`disablepositiveacksqospolicy` |
+--------------------------------------------------------------------------------+-------------------------------------+
| |RTPSReliableWriterQos::disable_heartbeat_piggyback-api|                       | :ref:`disableheartbeatpiggyback`    |
+--------------------------------------------------------------------------------+-------------------------------------+

* |RTPSReliableWriterQos::times-api|:
  Defines the duration of the RTPSWriter events.
  See :ref:`writertimes` for further details.
* |RTPSReliableWriterQos::disable_positive_acks-api|:
  Configures the settings to disable the positive acks.
  See :ref:`disablepositiveacksqospolicy` for further details.
* |RTPSReliableWriterQos::disable_heartbeat_piggyback-api|:
  Configures the settings to disable the heartbeat piggyback mechanism.
  See :ref:`disableheartbeatpiggyback` for further details.

.. note::
     This QoS Policy concerns to |DataWriter| entities.
     :raw-html:`<br />`
     Only the |DisablePositiveACKsQosPolicy::duration-api| Data Member of the :ref:`disablepositiveacksqospolicy` and the |RTPSReliableWriterQos::times-api|
     Data Member can be modified on enabled entities.

.. _writertimes:

WriterTimes
"""""""""""

This structure defines the times associated with the Reliable Writers' events.

List of structure members:

+-----------------------------------------------------------------------------------+------------------+---------------+
| Member Name                                                                       | Type             | Default Value |
+===================================================================================+==================+===============+
| |WriterTimes::initialHeartbeatDelay-api|                                          | |Duration_t-api| | 12ms          |
+-----------------------------------------------------------------------------------+------------------+---------------+
| |WriterTimes::heartbeatPeriod-api|                                                | |Duration_t-api| | 3s            |
+-----------------------------------------------------------------------------------+------------------+---------------+
| |WriterTimes::nackResponseDelay-api|                                              | |Duration_t-api| | 5ms           |
+-----------------------------------------------------------------------------------+------------------+---------------+
| |WriterTimes::nackSupressionDuration-api|                                         | |Duration_t-api| | 0s            |
+-----------------------------------------------------------------------------------+------------------+---------------+

* |WriterTimes::initialHeartbeatDelay-api|:
  Defines duration of the initial heartbeat delay.
* |WriterTimes::heartbeatPeriod-api|:
  Specifies the interval between periodic heartbeats.
* |WriterTimes::nackResponseDelay-api|:
  Establishes the duration of the delay applied to the response of an ACKNACK message.
* |WriterTimes::nackSupressionDuration-api|:
  The RTPSWriter ignores the nack messages received after sending the data until the
  duration time elapses.

.. _disableheartbeatpiggyback:

DisableHeartbeatPiggyback
"""""""""""""""""""""""""

Besides sending heartbeats periodically using the |WriterTimes::heartbeatPeriod-api| (see :ref:`writertimes`), reliable
DataWriters also use a mechanism to append a heartbeat submessage in the same message where data is being delivered to
the DataReaders.
This mechanism acts in specific situations where the reliable communication state must be up to date to maintain
optimal communication:

- When the DataWriter sends as many bytes to the *socket* as the length of the *socket* buffer, a heartbeat
  submessage is appended after the last data.
- When the DataWriter's history is full, the DataWriter starts to append heartbeat submessages after each data.

This mechanism can be disabled using this policy.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_RTPS_RELIABLE_WRITER_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_RTPS_RELIABLE_WRITER_QOS<-->
    :end-before: <!--><-->

.. _transportconfigqos:

TransportConfigQos
^^^^^^^^^^^^^^^^^^

This QoS Policy allows the configuration of the transport layer settings.
See |TransportConfigQos-api|.

List of QoS Policy data members:

.. list-table::
   :header-rows: 1

   * - Data Member Name
     - Type
     - Default Value
   * - |TransportConfigQos::user_transports-api|
     - ``std::vector<std::shared_ptr<TransportDescriptorInterface>>``
     - Empty vector
   * - |TransportConfigQos::use_builtin_transports-api|
     - ``bool``
     - ``true``
   * - |TransportConfigQos::send_socket_buffer_size-api|
     - ``uint32_t``
     - 0
   * - |TransportConfigQos::listen_socket_buffer_size-api|
     - ``uint32_t``
     - 0


* |TransportConfigQos::user_transports-api|:
  This data member defines the list of transports to use alongside or in place of builtins.
* |TransportConfigQos::use_builtin_transports-api|:
  It controls whether the built-in transport layer is enabled or disabled. If it is set to
  false, the default UDPv4 implementation is disabled.
* |TransportConfigQos::send_socket_buffer_size-api|:
  By default, Fast DDS creates socket buffers using the system default size. This data
  member allows to change the send socket buffer size used to send data.
* |TransportConfigQos::listen_socket_buffer_size-api|:
  The listen socket buffer size is also created with the system default size, but it can
  be changed using this data member.

.. note::
     This QoS Policy concerns to |DomainParticipant| entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _transportdescriptorinterface:

TransportDescriptorInterface
""""""""""""""""""""""""""""

This structure is the base for the data type used to define transport configuration.

List of structure members:

+--------------------------------------------------------------------------------------------------+-------------------+
| Member Name                                                                                      | Type              |
+==================================================================================================+===================+
| |TransportDescriptorInterface::maxMessageSize-api|                                               | ``uint32_t``      |
+--------------------------------------------------------------------------------------------------+-------------------+
| |TransportDescriptorInterface::maxInitialPeersRange-api|                                         | ``uint32_t``      |
+--------------------------------------------------------------------------------------------------+-------------------+

* |TransportDescriptorInterface::maxMessageSize-api|:
  This member sets the maximum size in bytes of the transport's message buffer.
* |TransportDescriptorInterface::maxInitialPeersRange-api|:
  This member states the maximum number of guessed initial peers to try to connect.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_TRANSPORT_CONFIG_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-COMMON-TRANSPORT-SETTING<-->
    :end-before: <!--><-->

.. note::
     :ref:`transportconfigqos` can also be configured modifying the builtin
     transports configuration by selecting one of the available builtin transports options.
     See :ref:`rtps_layer_builtin_transports` or |DomainParticipantQoS::setup_transports-api|.

.. _typeconsistencyqos:

TypeConsistencyQos
^^^^^^^^^^^^^^^^^^

This QoS Policy allows the configuration of the :ref:`XTypes extension QoS<xtypes_extensions>` on the |DataReader|.
See |TypeConsistencyQos-api|.

List of QoS Policy data members:

+-------------------------------------------------------------------------+--------------------------------------------+
| Data Member Name                                                        | Type                                       |
+=========================================================================+============================================+
| |TypeConsistencyQos::type_consistency-api|                              | :ref:`typeconsistencyenforcementqospolicy` |
+-------------------------------------------------------------------------+--------------------------------------------+
| |TypeConsistencyQos::representation-api|                                | :ref:`datarepresentationqospolicy`         |
+-------------------------------------------------------------------------+--------------------------------------------+

* |TypeConsistencyQos::type_consistency-api|:
  It states the rules for the data types compatibility.
  See :ref:`typeconsistencyenforcementqospolicy` for further details.
* |TypeConsistencyQos::representation-api|:
  It specifies the data representations valid for the entities.
  See :ref:`datarepresentationqospolicy` for further details.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_TYPE_CONSISTENCY_QOS
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML for the moment.

.. _wireprotocolconfigqos:

WireProtocolConfigQos
^^^^^^^^^^^^^^^^^^^^^

This QoS Policy allows the configuration of the wire protocol.
See |WireProtocolConfigQos-api|.

List of QoS Policy data members:

.. list-table::
    :header-rows: 1
    :align: left

    * - Data Member Name
      - Type
      - Default Value
    * - |WireProtocolConfigQos::prefix-api|
      - |GuidPrefix_t-api|
      - 0
    * - |WireProtocolConfigQos::participant_id-api|
      - ``int32_t``
      - -1
    * - |WireProtocolConfigQos::builtin-api|
      - |BuiltinAttributes-api|
      -
    * - |WireProtocolConfigQos::port-api|
      - |PortParameters-api|
      -
    * - |WireProtocolConfigQos::throughput_controller-api|
      - :ref:`throughputcontrollerdescriptor`
      -
    * - |WireProtocolConfigQos::default_unicast_locator_list-api|
      - |LocatorList_t-api|
      - Empty List
    * - |WireProtocolConfigQos::default_multicast_locator_list-api|
      - |LocatorList_t-api|
      - Empty List

* |WireProtocolConfigQos::prefix-api|:
  This data member allows the user to set manually the GUID prefix.
* |WireProtocolConfigQos::participant_id-api|:
  It sets the participant identifier. By default, it will be automatically generated by the Domain.
* |WireProtocolConfigQos::builtin-api|:
  This data member allows the configuration of the built-in parameters.
* |WireProtocolConfigQos::port-api|:
  This data member allows the configuration of the port parameters and gains related to the RTPS protocol
  (:ref:`listening_locators_defaultPorts`).
* |WireProtocolConfigQos::throughput_controller-api|:
  It allows the configuration of the throughput settings.
* |WireProtocolConfigQos::default_unicast_locator_list-api|:
  States the default list of unicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without unicast locators. This list should include at
  least one locator.
* |WireProtocolConfigQos::default_multicast_locator_list-api|:
  Stores the default list of multicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without multicast locators. This list is usually left
  empty.

.. note::
     This QoS Policy concerns to DomainParticipant entities.

.. important::
     The only mutable field on enabled entities is |m_DiscoveryServers|, which is contained in
     |BuiltinAttributes::discovery_config-api| within |WireProtocolConfigQos::builtin-api| (see
     :ref:`DS_modify_server_list`).

.. _throughputcontrollerdescriptor:

ThroughputControllerDescriptor
"""""""""""""""""""""""""""""""

This structure allows to limit the output bandwidth.
See |ThroughputControllerDescriptor-api|.

List of structure members:

+-------------------------------------------------------------------------------------------------+--------------------+
| Member Name                                                                                     | Type               |
+=================================================================================================+====================+
| |ThroughputControllerDescriptor::bytesPerPeriod-api|                                            | ``uint32_t``       |
+-------------------------------------------------------------------------------------------------+--------------------+
| |ThroughputControllerDescriptor::periodMillisecs-api|                                           | ``uint32_t``       |
+-------------------------------------------------------------------------------------------------+--------------------+

* |ThroughputControllerDescriptor::bytesPerPeriod-api|:
  This member states the number of bytes that this controller will allow in a given period.
* |ThroughputControllerDescriptor::periodMillisecs-api|:
  It specifies the window of time in which no more than `bytesPerPeriod` bytes are allowed.

.. warning::
    This has been deprecated in favor of |FlowControllersQos|

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_WIRE_PROTOCOL_CONFIG_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_WIRE_PROTOCOL_CONFIG_QOS<-->
    :end-before: <!--><-->

.. _writerresourcelimitsqos:

WriterResourceLimitsQos
^^^^^^^^^^^^^^^^^^^^^^^

This QoS Policy states the limits for the matched |DataReaders|' resource limited collections based on the maximum
number of DataReaders that are going to match with the |DataWriter|.
See |WriterResourceLimitsQos-api|.

List of QoS Policy data members:

+-----------------------------------------------------------------------------+----------------------------------------+
| Data Member Name                                                            | Type                                   |
+=============================================================================+========================================+
| |WriterResourceLimitsQos::matched_subscriber_allocation-api|                | :ref:`resourcelimitedcontainerconfig`  |
+-----------------------------------------------------------------------------+----------------------------------------+
| |WriterResourceLimitsQos::reader_filters_allocation-api|                    | :ref:`resourcelimitedcontainerconfig`  |
+-----------------------------------------------------------------------------+----------------------------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_WRITER_RESOURCE_LIMITS_QOS
   :end-before: //!

XML
***
.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML_WRITER_RESOURCE_LIMITS_QOS<-->
    :end-before: <!--><-->
