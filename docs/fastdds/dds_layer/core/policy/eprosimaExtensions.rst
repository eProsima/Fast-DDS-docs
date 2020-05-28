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

.. _disablepositiveacksqospolicy:

DisablePositiveACKsQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This additional QoS allows reducing network traffic when strict reliable communication is not required and bandwidth is
limited.
It consists in changing the default behavior by which positive acks are sent from readers to writers.
Instead, only negative acks will be sent when a reader is missing a sample, but writers will keep data for a sufficient
time before considering it as acknowledged.

List of QoS Policy data members:

+------------------------------------+------------------------------+-------------------+
| Data Member Name                   | Type                         | Default Value     |
+====================================+==============================+===================+
| enabled                            | bool                         | false             |
+------------------------------------+------------------------------+-------------------+
| duration                           | fastrtps::Duration_t         | c_TimeInfinite    |
+------------------------------------+------------------------------+-------------------+

* **Enabled**: Specifies if the QoS is enabled or not. If it is true means that the positive acks are disabled and the
  DataReader only sends negative acks. Otherwise, both positive and negative acks are sent.
* **Duration**: State the duration that the DataWriters keep the data before considering it as acknowledged.
  This value does not apply to DataReaders.

.. note::
     This QoS Policy concerns to DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. warning::
    For DataWriters and DataReaders to match, they must follow the compatibility rule. See :ref:`disableacks_compatibilityrule`
    for further details.

.. _disableacks_compatibilityrule:

Compatibility Rule
""""""""""""""""""

To maintain the compatibility between DisablePositiveACKsQosPolicy in DataReaders and DataWriters, the DataReader
cannot have this QoS enabled if the DataWriter have it disabled.

Table with the possible combinations:

+----------------------------+----------------------------+-----------------+
| DataWriter `enabled` value | DataReader `enabled` value | Compatibility   |
+============================+============================+=================+
| true                       | true                       | Yes             |
+----------------------------+----------------------------+-----------------+
| true                       | false                      | Yes             |
+----------------------------+----------------------------+-----------------+
| false                      | true                       | No              |
+----------------------------+----------------------------+-----------------+
| false                      | false                      | Yes             |
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

.. _participantresourcelimitsqos:

ParticipantResourceLimitsQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This QoS configures allocation limits and the use of physical memory for internal resources.

List of QoS Policy data members:

+-------------------------+-------------------------------------------+
| Data Member Name        | Type                                      |
+=========================+===========================================+
| locators                | :ref:`remotelocatorsallocationattributes` |
+-------------------------+-------------------------------------------+
| participants            | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------+-------------------------------------------+
| readers                 | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------+-------------------------------------------+
| writers                 | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------+-------------------------------------------+
| send_buffers            | :ref:`sendbuffersallocationattributes`    |
+-------------------------+-------------------------------------------+
| data_limits             | :ref:`variablelengthdatalimits`           |
+-------------------------+-------------------------------------------+

* **Locators**: Defines the limits for collections of remote locators.
* **Participants**: Specifies the allocation behavior and limits for collections dependent on the total number of
  participants.
* **Readers**: Specifies the allocation behavior and limits for collections dependent on the total number of
  readers per participant.
* **Writers**: Specifies the allocation behavior and limits for collections dependent on the total number of
  writers per participant.
* **Send buffers**: Defines the allocation behavior and limits for the send buffer manager.
* **Data Limits**: States the limits for variable-length data.

.. note::
     This QoS Policy concerns to DomainParticipant entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _remotelocatorsallocationattributes:

RemoteLocatorsAllocationAttributes
""""""""""""""""""""""""""""""""""

This structure holds the limits for the remote locators' collections.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| max_unicast_locators               | size_t                       | 4                 |
+------------------------------------+------------------------------+-------------------+
| max_multicast_locators             | size_t                       | 1                 |
+------------------------------------+------------------------------+-------------------+

* **Max unicast locators**: This member controls the maximum number of unicast locators to keep for each discovered
  remote entity.
  It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain.
* **Max multicast locators**: This member controls the maximum number of multicast locators to keep for each discovered
  remote entity.
  The default value is usually enough, as it doesn't make sense to add more than one multicast locator per entity.


.. _resourcelimitedcontainerconfig:

ResourceLimitedContainerConfig
""""""""""""""""""""""""""""""

This structure holds the limits of a resource limited collection, as well as the allocation configuration, which can be
fixed size or dynamic size.

List of structure members:

+------------------------------------+------------------------------+-----------------------------------+
| Member Name                        | Type                         | Default Value                     |
+====================================+==============================+===================================+
| initial                            | size_t                       | 0                                 |
+------------------------------------+------------------------------+-----------------------------------+
| maximum                            | size_t                       | std::numeric_limits<size_t>::max()|
+------------------------------------+------------------------------+-----------------------------------+
| increment                          | size_t                       | 1 (dynamic size), 0 (fixed size)  |
+------------------------------------+------------------------------+-----------------------------------+

* **Initial**: Indicates the number of elements to preallocate in the collection.
* **Maximum**: Specifies the maximum number of elements allowed in the collection.
* **Increment**: States the number of items to add when the reserved capacity limit is reached. This member has a
  different default value depending on the allocation configuration chosen.

.. _sendbuffersallocationattributes:

SendBuffersAllocationAttributes
"""""""""""""""""""""""""""""""

This structure holds the limits for the allocations of the send buffers.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| preallocated_number                | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+
| dynamic                            | bool                         | false             |
+------------------------------------+------------------------------+-------------------+

* **Preallocated number**: This member controls the initial number of send buffers to be allocated.
  The default value will perform an initial guess of the number of buffers required, based on the number of threads
  from which a send operation could be started.
* **Dynamic**: This member controls how the buffer manager behaves when a send buffer is not available.
  When true, a new buffer will be created. Otherwise, it will wait for a buffer to be returned.

.. _variablelengthdatalimits:

VariableLengthDataLimits
""""""""""""""""""""""""

This structure holds the limits for variable-length data.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| max_properties                     | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+
| max_user_data                      | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+
| max_partitions                     | size_t                       | 0                 |
+------------------------------------+------------------------------+-------------------+

* **Max properties**: Defines the maximum size, in octets, of the properties data in the local or remote participant.
* **Max user data**: Establishes the maximum size, in octets, of the user data in the local or remote participant.
* **Max partitions**: States the maximum size, in octets, of the partitions data in the local or remote participant.

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
    :end-before: <publisher

.. _propertypolicyqos:

PropertyPolicyQos
^^^^^^^^^^^^^^^^^

This additional QoS Policy stores name/value pairs that can be used to configure certain DDS settings that cannot
be configured directly using an standard QoS Policy.
In Fast DDS, it can be used to configure the security settings (See :ref:`security` for further details of the security
functionality).

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

This QoS Policy configures how the DataWriter sends the data.

List of QoS Policy data members:

+--------------------------+--------------------------------+-----------------------+
| Data Member Name         | Type                           | Default Value         |
+==========================+================================+=======================+
| kind                     | :ref:`publishmodeqospolicykind`| SYNCHRONOUS           |
+--------------------------+--------------------------------+-----------------------+

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _publishmodeqospolicykind:

PublishModeQosPolicyKind
""""""""""""""""""""""""

There are two possible values:

* ``SYNCHRONOUS_PUBLISH_MODE``: The data is sent in the context of the user thread that calls the write operation.
* ``ASYNCHRONOUS_PUBLISH_MODE``: An internal thread takes the responsibility of sending the data asynchronously.
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

This QoS Policy states the limits for the matched DataWriters' resource limited collections based on the maximum number
of DataWriters that are going to match with the DataReader.

List of QoS Policy data members:

+------------------------------+-------------------------------------------+
| Data Member Name             | Type                                      |
+==============================+===========================================+
| matched_publisher_allocation | :ref:`resourcelimitedcontainerconfig`     |
+------------------------------+-------------------------------------------+


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

List of QoS Policy data members:

+--------------------------+--------------------------------+-----------------------+
| Data Member Name         | Type                           | Default Value         |
+==========================+================================+=======================+
| unicast_locator_list     | fastrtps::rtps::LocatorList_t  | Empty List            |
+--------------------------+--------------------------------+-----------------------+
| multicast_locator_list   | fastrtps::rtps::LocatorList_t  | Empty List            |
+--------------------------+--------------------------------+-----------------------+
| remote_locator_list      | fastrtps::rtps::LocatorList_t  | Empty List            |
+--------------------------+--------------------------------+-----------------------+
| user_defined_id          | int16_t                        | -1                    |
+--------------------------+--------------------------------+-----------------------+
| entity_id                | int16_t                        | -1                    |
+--------------------------+--------------------------------+-----------------------+
| history_memory_policy    | :ref:`memorymanagementpolicy`  | PREALLOCATED          |
+--------------------------+--------------------------------+-----------------------+

* **Unicast locator list**: Defines the list of unicast locators associated to the DDS Entity.
  DataReaders and DataWriters inherit the list of unicast locators set in the DomainParticipant, but it can be
  changed by means of this QoS.
* **Multicast locator list**: Stores the list of multicast locators associated to the DDS Entity.
  By default, DataReaders and DataWriters don't use any multicast locator, but it can be changed by means of this QoS.
* **Remote locator list**: States the list of remote locators associated to the DDS Entity.
* **User defined ID**: Establishes the unique identifier used for StaticEndpointDiscovery.
* **Entity ID**: The user can specify the identifier for the endpoint.
* **History memory policy**: Indicates the way the memory is managed in terms of dealing with the CacheChanges.

.. note::
     This QoS Policy concerns to DataWriter and DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _memorymanagementpolicy:

MemoryManagementPolicy
""""""""""""""""""""""

There are four possible values:

* ``PREALLOCATED_MEMORY_MODE``: This option sets the size to the maximum of each data type. It produces the largest
  memory footprint but the smallest allocation count.
* ``PREALLOCATED_WITH_REALLOC_MEMORY_MODE``: This option set the size to the default for each data type and it requires
  reallocation when a bigger message arrives. It produces a lower memory footprint at the expense of increasing the
  allocation count.
* ``DYNAMIC_RESERVE_MEMORY_MODE``: This option allocates the size dynamically at the time of message arrival. It
  produces the least memory footprint but the highest allocation count.
* ``DYNAMIC_REUSABLE_MEMORY_MODE``: This option is similar to ``DYNAMIC_RESERVE_MEMORY_MODE``, but the allocated memory
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

List of QoS Policy data members:

+--------------------------+------------------------------------+
| Data Member Name         | Type                               |
+==========================+====================================+
| times                    | :ref:`readertimes`                 |
+--------------------------+------------------------------------+
| disable_positive_ACKs    | :ref:`disablepositiveacksqospolicy`|
+--------------------------+------------------------------------+

* **Times**: Defines the duration of the RTPSReader events. See :ref:`readertimes` for further details.
* **Disable positive ACKs**: Configures the settings to disable the positive acks.
  See :ref:`disablepositiveacksqospolicy` for further details.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _readertimes:

ReaderTimes
"""""""""""

This structure defines the times associated with the Reliable Readers' events.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| initialAcknackDelay                | fastrtps::Duration_t         | 70 milliseconds   |
+------------------------------------+------------------------------+-------------------+
| heartbeatResponseDelay             | fastrtps::Duration_t         | 5 milliseconds    |
+------------------------------------+------------------------------+-------------------+

* **Initial acknack delay**: Defines the duration of the initial acknack delay.
* **Heartbeat response delay**: Establishes the duration of the delay applied when a heartbeat message is received.

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

List of QoS Policy data members:

+--------------------------+------------------------------------+
| Data Member Name         | Type                               |
+==========================+====================================+
| times                    | :ref:`writertimes`                 |
+--------------------------+------------------------------------+
| disable_positive_acks    | :ref:`disablepositiveacksqospolicy`|
+--------------------------+------------------------------------+

* **Times**: Defines the duration of the RTPSWriter events. See :ref:`writertimes` for further details.
* **Disable positive ACKs**: Configures the settings to disable the positive acks.
  See :ref:`disablepositiveacksqospolicy` for further details.

.. note::
     This QoS Policy concerns to DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _writertimes:

WriterTimes
"""""""""""

This structure defines the times associated with the Reliable Writers' events.

List of structure members:

+------------------------------------+------------------------------+-------------------+
| Member Name                        | Type                         | Default Value     |
+====================================+==============================+===================+
| initialHeartbeatDelay              | fastrtps::Duration_t         | 12 milliseconds   |
+------------------------------------+------------------------------+-------------------+
| heartbeatPeriod                    | fastrtps::Duration_t         | 3 seconds         |
+------------------------------------+------------------------------+-------------------+
| nackResponseDelay                  | fastrtps::Duration_t         | 5 milliseconds    |
+------------------------------------+------------------------------+-------------------+
| nackSupressionDuration             | fastrtps::Duration_t         | 0 seconds         |
+------------------------------------+------------------------------+-------------------+

* **Initial heartbeat delay**: Defines duration of the initial heartbeat delay.
* **Heartbeat period**: Specifies the interval between periodic heartbeats.
* **Nack response delay**: Establishes the duration of the delay applied to the response of an ACKNACK message.
* **Nack supression duration**: The RTPSWriter ignores the nack messages received after sending the data until the
  duration time elapses.

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

List of QoS Policy data members:

+---------------------------+------------------------------------------------------------------+-----------------+
| Data Member Name          | Type                                                             | Default Value   |
+===========================+==================================================================+=================+
| user_transports           | std::vector<std::shared_ptr<:ref:`transportdescriptorinterface`>>| Empty Vector    |
+---------------------------+------------------------------------------------------------------+-----------------+
| use_builtin_transports    | bool                                                             | true            |
+---------------------------+------------------------------------------------------------------+-----------------+
| send_socket_buffer_size   | uint32_t                                                         | 0               |
+---------------------------+------------------------------------------------------------------+-----------------+
| listen_socket_buffer_size | uint32_t                                                         | 0               |
+---------------------------+------------------------------------------------------------------+-----------------+

* **User transports**: This data member defines the list of transports to use alongside or in place of builtins.
* **Use builtin transports**: It controls whether the built-in transport layer is enabled or disabled. If it is set to
  false, the default UDPv4 implementation is disabled.
* **Send socket buffer size**: By default, Fast DDS creates socket buffers using the system default size. This data
  member allows to change the send socket buffer size used to send data.
* **Listen socket buffer size**: The listen socket buffer size is also created with the system default size, but it can
  be changed using this data member.

.. note::
     This QoS Policy concerns to DomainParticipant entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _transportdescriptorinterface:

TransportDescriptorInterface
""""""""""""""""""""""""""""

This structure is the base for the data type used to define transport configuration.

List of structure members:

+------------------------------------+------------------------------+
| Member Name                        | Type                         |
+====================================+==============================+
| maxMessageSize                     | uint32_t                     |
+------------------------------------+------------------------------+
| maxInitialPeersRange               | uint32_t                     |
+------------------------------------+------------------------------+

* **Max message size**: This member sets the maximum size in bytes of the transport's message buffer.
* **Max initial peers range**: This member states the maximum number of guessed initial peers to try to connect.

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

.. _typeconsistencyqos:

TypeConsistencyQos
^^^^^^^^^^^^^^^^^^

This QoS Policy allows the configuration of the XTypes extension QoS on the DataReader.

List of QoS Policy data members:

+--------------------------------+-------------------------------------------+
| Data Member Name               | Type                                      |
+================================+===========================================+
| type_consistency               | :ref:`typeconsistencyenforcementqospolicy`|
+--------------------------------+-------------------------------------------+
| representation                 | :ref:`datarepresentationqospolicy`        |
+--------------------------------+-------------------------------------------+

* **Type consistency**: It states the rules for the data types compatibility.
  See :ref:`typeconsistencyenforcementqospolicy` for further details.
* **Representation**: It specifies the data representations valid for the entities.
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

List of QoS Policy data members:

+--------------------------------+---------------------------------------+-----------------------+
| Data Member Name               | Type                                  | Default Value         |
+================================+=======================================+=======================+
| prefix                         | fastrtps::rtps::GuidPrefix_t          | 0                     |
+--------------------------------+---------------------------------------+-----------------------+
| participant_id                 | int32_t                               | -1                    |
+--------------------------------+---------------------------------------+-----------------------+
| builtin                        | :ref:`DS_BuiltinAttributes`           |                       |
+--------------------------------+---------------------------------------+-----------------------+
| throughput_controller          | :ref:`throughputcontrollerdescription`|                       |
+--------------------------------+---------------------------------------+-----------------------+
| default_unicast_locator_list   | fastrtps::rtps::LocatorList_t         | Empty List            |
+--------------------------------+---------------------------------------+-----------------------+
| default_multicast_locator_list | fastrtps::rtps::LocatorList_t         | Empty List            |
+--------------------------------+---------------------------------------+-----------------------+

* **Prefix**: This data member allows the user to set manually the GUID prefix.
* **Participant ID**: It sets the participant identifier. By default, it will be automatically generated by the Domain.
* **Builtin**: This data member allows the configuration of the built-in parameters.
  See :ref:`DS_BuiltinAttributes` for further details.
* **Throughput controller**: It allows the configuration of the throughput settings.
* **Default unicast locator list**: States the default list of unicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without unicast locators. This list should include at
  least one locator.
* **Default multicast locator list**: Stores the default list of multicast locators to be used for any endpoint defined
  inside the RTPSParticipant in the case that it was defined without multicast locators. This list is usually left
  empty.

.. note::
     This QoS Policy concerns to DomainParticipant entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _throughputcontrollerdescription:

ThroughputControllerDescription
"""""""""""""""""""""""""""""""

This structure allows to limit the output bandwidth.

List of structure members:

+------------------------------------+------------------------------+
| Member Name                        | Type                         |
+====================================+==============================+
| bytesPerPeriod                     | uint32_t                     |
+------------------------------------+------------------------------+
| periodMillisecs                    | uint32_t                     |
+------------------------------------+------------------------------+

* **Bytes per period**: This member states the number of bytes that this controller will allow in a given period.
* **Period in milliseconds**: It specifies the window of time in which no more than `bytesPerPeriod` bytes are allowed.

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

This QoS Policy states the limits for the matched DataReaders' resource limited collections based on the maximum number
of DataReaders that are going to match with the DataWriter.

List of QoS Policy data members:

+-------------------------------+-------------------------------------------+
| Data Member Name              | Type                                      |
+===============================+===========================================+
| matched_subscriber_allocation | :ref:`resourcelimitedcontainerconfig`     |
+-------------------------------+-------------------------------------------+

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
