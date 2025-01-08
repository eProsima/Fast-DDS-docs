.. include:: ../03-exports/aliases.include
.. include:: ../03-exports/aliases-api.include
.. include:: ../03-exports/roles.include

.. _migration_guide:

Migration Guide to Fast DDS v3
==============================

This document aims to help during the migration process from eProsima *Fast DDS version* 2 to *Fast DDS version* 3.
For more information about all the updates, please refer to the :ref:`release notes <release_notes>`.

Migration Steps
---------------

The following steps describe the possible changes that your project may require to migrate to *Fast DDS v3.0.0*:

- :ref:`step-1-update-the-package-name-and-cmake-configuration`
- :ref:`step-2-update-dependencies`
- :ref:`step-3-ensure-compatibility-with-related-products`
- :ref:`step-4-apply-namespace-changes`
- :ref:`step-5-migrate-public-headers`
- :ref:`step-6-handle-removed-or-private-headers`
- :ref:`step-7-update-api-methods`
- :ref:`step-8-update-structs-enums-and-variables`
- :ref:`step-9-refactor-examples`

.. _step-1-update-the-package-name-and-cmake-configuration:

Step 1: Update the package name and CMake configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. CMake project name: Rename all instances in the CMake project from ``fastrtps`` to ``fastdds``. For example, 
   update ``target_link_libraries(fastrtps)`` to ``target_link_libraries(fastdds)``, and ``if(NOT fastrtps_FOUND)`` to
   ``if(NOT fastdds_FOUND)``.
2. Environment variables:

   * Rename ``FASTRTPS_DEFAULT_PROFILES_FILE`` to ``FASTDDS_DEFAULT_PROFILES_FILE``.
   * The configuration file for loading profiles has been renamed from ``DEFAULT_FASTRTPS_PROFILES.xml`` to 
     ``DEFAULT_FASTDDS_PROFILES.xml``.

.. _step-2-update-dependencies:

Step 2: Update dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS v3 is only compatible with Fast CDR v2.
If you are not using Fast CDR as :ref:`third-party <third-party_libraries-options>`, please ensure that your local
dependencies are up-to-date.
Refer to the :ref:`library deprendencies table <dependencies_compatibilities_build_system_dependencies>` to verify
version compatibility for all Fast DDS library dependencies.

.. _step-3-ensure-compatibility-with-related-products:

Step 3: Ensure compatibility with related products
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS v3 requires Fast DDS Gen v4 for code generation. Make sure to regenerate types using this compatible version.

For other compatibility requirements with related products (e.g., Shapes Demo, Discovery Server), refer to the table
of :ref:`products compatibility <dependencies_compatibilities_product_compatibility>`, which outlines version
compatibility across the Fast DDS ecosystem.
Verify these versions and update accordingly to avoid any integration issues.

.. _step-4-apply-namespace-changes:

Step 4: Apply namespace changes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Namespace migration:

   * First, update all ``eprosima::fastrtps::`` namespace references to ``eprosima::fastdds::``.
   * Move built-in topics ``SubscriptionBuiltinTopicData``, ``PublicationBuiltinTopicData``, and
     ``ParticipantBuiltinTopicData`` from ``eprosima::fastdds::dds::builtin::`` to ``eprosima::fastdds::rtps::``.
   * Move ``Duration_t`` and ``c_TimeInfinite`` references from ``eprosima::fastdds::`` to ``eprosima::fastdds::dds``.
   * Move ``Time_t.hpp`` references from ``eprosima::fastdds::`` to ``eprosima::fastdds::dds``.

   Ensure you update these namespace references across your code to avoid compilation errors.

2. Renamed types:

   * Change ``EventKindBits::`` references to ``EventKind::``.
   * Change ``EventKindEntityId::`` references to ``EntityId::``.
   * Change ``StatisticsEventKind::`` references to ``statistics::EventKind::``.

   Refactor the type references as outlined above to maintain compatibility with the new version.

.. _step-5-migrate-public-headers:

Step 5: Migrate public headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Header files location:

   All the headers in ``include/fastrtps`` were migrated to ``include/fastdds``.
   In particular, the following list includes headers that have been relocated to different paths or whose
   implementations have been incorporated into other headers:

   .. list-table::
      :header-rows: 1

      * - Fast DDS v2 file *include* path
        - Fast DDS v3 file *include* path
      * - ``fastdds/rtps/resources/ResourceManagement.hpp``
        - ``fastdds/rtps/attributes/ResourceManagement.hpp``
      * - ``fastrtps/eProsima_auto_link.h``
        - ``fastdds/fastdds_auto_link.hpp``
      * - ``fastrtps/attributes/ParticipantAttributes.h``
        - ``fastdds/dds/domain/qos/DomainParticipantExtendedQos.hpp``
      * - ``fastrtps/Domain.h``
        - ``fastdds/dds/domain/DomainParticipantFactory.hpp``
      * - ``fastrtps/log/Log.h``
        - ``fastdds/dds/log/Log.hpp``
      * - ``fastrtps/qos/DeadlineMissedStatus.h``
        - ``fastdds/dds/core/status/DeadlineMissedStatus.hpp``
      * - ``fastrtps/qos/IncompatibleQosStatus.hpp``
        - ``fastdds/dds/core/status/IncompatibleQosStatus.hpp``
      * - ``fastrtps/qos/LivelinessChangedStatus.h``
        - ``fastdds/dds/core/status/LivelinessChangedStatus.hpp``
      * - ``fastrtps/qos/QosPolicies.h``
        - ``fastdds/dds/core/policy/QosPolicies.hpp``
      * - ``fastrtps/qos/ReaderQos.h``
        - ``fastdds/dds/subscriber/qos/ReaderQos.hpp``
      * - ``fastrtps/qos/WriterQos.h``
        - ``fastdds/dds/publisher/qos/WriterQos.hpp``
      * - ``fastrtps/qos/SampleRejectedStatus.hpp``
        - ``fastdds/dds/core/status/SampleRejectedStatus.hpp``
      * - ``fastrtps/participant/Participant.h``
        - ``fastdds/rtps/participant/RTPSParticipant.hpp``
      * - ``fastrtps/transport/TCPv4TransportDescriptor.h``
        - ``fastdds/rtps/transport/TCPv4TransportDescriptor.hpp``
      * - ``fastrtps/transport/TCPv6TransportDescriptor.h``
        - ``fastdds/rtps/transport/TCPv6TransportDescriptor.hpp``
      * - ``fastrtps/transport/UDPv4TransportDescriptor.h``
        - ``fastdds/rtps/transport/UDPv4TransportDescriptor.hpp``
      * - ``fastrtps/transport/UDPv6TransportDescriptor.h``
        - ``fastdds/rtps/transport/UDPv6TransportDescriptor.hpp``
      * - ``fastrtps/transport/UDPTransportDescritpor.h``
        - ``fastdds/rtps/transport/UDPTransportDescritpor.hpp``
      * - ``fastrtps/transport/TCPTransportDescritpor.h``
        - ``fastdds/rtps/transport/TCPTransportDescritpor.hpp``
      * - ``fastdds/rtps/common/Time_t.hpp in namespace{fastdds}``
        - ``fastdds/dds/core/Time_t.hpp in namespace{fastdds::dds}``

   Also, the ``fixed_size_string.hpp`` implementation has been migrated from ``fastrtps/utils/fixed_size_string.hpp``
   to ``fastcdr/cdr/fixed_size_string.hpp``.

2. File extensions:

   Rename file extensions from `.h` to `.hpp`.

.. _step-6-handle-removed-or-private-headers:

Step 6: Handle removed or private headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following list contains headers that were previously in the `include` folder and have been relocated to
the `src/cpp` folder.
Since they are no longer public, it is not possible to include them in external projects:

* ParticipantAttributes.hpp
* ReplierAttributes.hpp
* RequesterAttributes.hpp
* PublisherAttributes.hpp
* SubscriberAttributes.hpp
* ProxyPool.hpp
* Semaphore.hpp
* MessageReceiver.hpp
* BuiltinProtocols.hpp
* shared_mutex.hpp
* StringMatching.hpp
* TimeConversion.hpp
* DBQueue.hpp
* ResourceEvent.hpp
* TimedEvent.hpp
* WriterProxyData.hpp
* ReaderProxyData.hpp
* ParticipantProxyData.hpp
* XML Parser API
* UnitsParser.hpp
* RTPSMessageGroup.hpp
* RTPSMessageCreator.hpp
* CDRMessage.hpp
* StatefulPersistentReader.hpp
* StatefulReader.hpp
* StatelessPersistentReader.hpp
* StatelessReader.hpp
* PersistentWriter.hpp
* StatefulPersistentWriter.hpp
* StatefulWriter.hpp
* StatelessPersistentWriter.hpp
* StatelessWriter.hpp
* logging.h
* Exception.h
* Cryptography.h
* Authentication.h
* AccessControl.h
* SecurityException.h
* ChangeForReader.hpp
* ReaderLocator.hpp
* ReaderProxy.hpp
* ServerAttributes.hpp
* TopicAttributes.hpp
* TypeLookupService.hpp

If your project previously included any of these headers, you will need to modify your implementation.
Since these headers are now private, you should replace their usage with public alternatives or refactor the
related code to ensure it does not depend on private headers.

.. TODO:: Add a note about which headers to use instead of the private ones, since we mention that they should 
  be replaced with public alternatives.
  
.. _step-7-update-api-methods:

Step 7: Update API methods
^^^^^^^^^^^^^^^^^^^^^^^^^^

The table below contains the list of API changes, showing the previous methods and the corresponding new ones
introduced in Fast DDS v3.
The new API methods achieve the same functionality, even though the signature of the method is different
from the deprecated one.

.. list-table::
   :header-rows: 1

   * - Deprecated methods
     - New methods
   * - ``xmlparser::XMLProfileManager::library_settings(LibrarySettingsAttributes&)``
     - ``DomainParticipantFactory::get_instance()->set_library_settings(const LibrarySettings&)``
   * - ``fill_discovery_data_from_cdr_message(ReaderProxyData&, MonitorServiceStatusData&)``
     - ``fill_discovery_data_from_cdr_message(SubscriptionBuiltinTopicData&, MonitorServiceStatusData&)``
   * - ``fill_discovery_data_from_cdr_message(WriterProxyData&, MonitorServiceStatusData&)``
     - ``fill_discovery_data_from_cdr_message(PublicationBuiltinTopicData&, MonitorServiceStatusData&)``
   * - ``fill_discovery_data_from_cdr_message(ParticipantProxyData&, MonitorServiceStatusData&)``
     - ``fill_discovery_data_from_cdr_message(ParticipantBuiltinTopicData&, MonitorServiceStatusData&)``
   * - ``on_participant_discovery(DomainParticipant*, ParticipantDiscoveryInfo&&, bool)``
     - ``on_participant_discovery(DomainParticipant*, ParticipantDiscoveryStatus, ParticipantBuiltinTopicData&, bool&)``
   * - ``on_subscriber_discovery(DomainParticipant*, ReaderDiscoveryInfo&&, bool)``
     - ``on_data_reader_discovery(DomainParticipant*, ReaderDiscoveryStatus, SubscriptionBuiltinTopicData&, bool&)``
   * - ``on_publisher_discovery(DomainParticipant*, WriterDiscoveryInfo&&, bool)``
     - ``on_data_writer_discovery(DomainParticipant*, WriterDiscoveryStatus, PublicationBuiltinTopicData&, bool&)``
   * - ``onReaderDiscovery(RTPSParticipant*, ReaderDiscoveryInfo&&, bool)``
     - ``on_reader_discovery(RTPSParticipant*, ReaderDiscoveryStatus, SubscriptionBuiltinTopicData&, bool&)``
   * - ``onWriterDiscovery(RTPSParticipant*, WriterDiscoveryInfo&&, bool)``
     - ``on_writer_discovery(RTPSParticipant*, WriterDiscoveryStatus, PublicationBuiltinTopicData&, bool&)``
   * - ``onParticipantDiscovery(RTPSParticipant*, ParticipantDiscoveryInfo&&, bool)``
     - ``on_participant_discovery(RTPSParticipant*, ParticipantDiscoveryStatus, ParticipantBuiltinTopicData&, bool&)``
   * - ``XMLProfileManager::loadXMLFile(string&)``
     - ``DomainParticipantFactory::get_instance()->load_XML_profiles_file(string)``
   * - ``XMLProfileManager::loadDefaultXMLFile()``
     - ``load_profiles()``
   * - ``XMLProfileManager::loadXMLFile(string)``
     - ``load_XML_profiles_file(string&)``
   * - ``XMLProfileManager::loadXMLString(const char*, size_t)``
     - ``load_XML_profiles_string(const char*, size_t)``
   * - ``XMLProfileManager::fillParticipantAttributes(const string&, ParticipantAttributes&, bool)``
     - ``get_participant_qos_from_profile(string&, DomainParticipantQos&)``
   * - ``DynamicTypeBuilder XMLProfileManager::getDynamicTypeByName(string&)``
     - ``get_dynamic_type_builder_from_xml_by_name(string&, DynamicTypeBuilder::_ref_type&)``
   * - ``XMLProfileManager::fillRequesterAttributes(string&, RequesterAttributes&)``
     - ``get_requester_qos_from_profile(string&, RequesterQos&)``
   * - ``XMLParser::getXMLThroughputController(tinyxml2::XMLElement*, ThroughputControllerDescriptor&, uint8_t)``
     - ``XMLParser::getXMLFlowControllerDescriptorList(tinyxml2::XMLElement*, FlowControllerDescriptorList&, uint8_t)``
   * - ``add_throughput_controller_descriptor_to_pparams(FlowControllerSchedulerPolicy, uint32_t, uint32_t)``
     - ``add_flow_controller_descriptor_to_pparams(FlowControllerSchedulerPolicy, uint32_t, uint32_t)``
   * - ``get_payload(uint32_t, CacheChange_t&)``
     - ``get_payload(uint32_t, SerializedPayload_t&)``
   * - ``release_payload(CacheChange_t&)``
     - ``release_payload(SerializedPayload_t&)``
   * - ``registerWriter(RTPSWriter*, const TopicAttributes&, const WriterQos&)``
     - ``register_writer(RTPSWriter*, const PublicationBuiltinTopicData&)``
   * - ``registerReader(RTPSReader*, TopicAttributes&, ReaderQos&)``
     - ``register_reader(RTPSReader*, const SubscriptionBuiltinTopicData&, const ContentFilterProperty*)``
   * - ``updateWriter(RTPSWriter*, const TopicAttributes&, const WriterQos&)``
     - ``update_writer(RTPSWriter*, const WriterQos&)``
   * - ``updateReader(RTPSReader*, const TopicAttributes&, const ReaderQos&, const ContentFilterProperty*)``
     - ``update_reader(RTPSReader*, const ReaderQos, const ContentFilterProperty*)``
   * - ``getRTPSParticipantAttributes()``
     - ``get_attributes()``
   * - ``bool write(void*)``
     - ``ReturnCode_t write(void*)``
   * - ``bool write(void*, WriteParams&)``
     - ``ReturnCode_t write(void*, WriteParams&)``
   * - ``SenderResource::send(const octet*, uint32_t, LocatorsIterator*, LocatorsIterator*, const chrono::steady_clock::time_point&)``
     - ``SenderResource::send(vector<NetworkBuffer>, uint32_t, LocatorsIterator*, LocatorsIterator*, const chrono::steady_clock::time_point&)``
   * - ``RTPSMessageSenderInterface::send(CDRMessage_t*, chrono::steady_clock::time_point)``
     - ``RTPSMessageSenderInterface::send(vector<NetworkBuffer>&, uint32_t&, chrono::steady_clock::time_point)``
   * - ``createRTPSWriter(RTPSParticipant*, EntityId_t&, WriterAttributes&, shared_ptr<IPayloadPool>&, shared_ptr<IChangePool>&, WriterHistory*, WriterListener*)``
     - ``createRTPSWriter(RTPSParticipant*, WriterAttributes&, WriterHistory*, WriterListener*)``
   * - ``RTPSWriter::new_change(const function<uint32_t()>& dataCdrSerializedSize, ChangeKind_t, InstanceHandle_t)``
     - ``WriterHistory::create_change(uint32_t, ChangeKind_t, InstanceHandle_t)``
   * - ``RTPSWriter::new_change(ChangeKind_t, InstanceHandle_t)``
     - ``WriterHistory::create_change(ChangeKind_t, InstanceHandle_t)``
   * - ``RTPSWriter::release_change(CacheChange_t*)``
     - ``WriterHistory::release_change(CacheChange_t*)``
   * - ``RTPSWriter::remove_older_changes(unsigned int)``
     - ``WriterHistory::remove_min_change()``
   * - ``RTPSWriter::is_acked_by_all(const CacheChange_t*)``
     - ``RTPSWriter::is_acked_by_all(const SequenceNumber_t&)``
   * - ``RTPSWriter::updateAttributes(const WriterAttributes&)``
     - ``RTPSWriter::update_attributes(const WriterAttributes&)``
   * - ``RTPSWriter::getListener()``
     - ``RTPSWriter::get_listener()``
   * - ``RTPSWriter::isAsync()``
     - ``RTPSWriter::is_async()``
   * - ``WriterListener::onWriterMatched(RTPSWriter*, MatchingInfo&)``
     - ``WriterListener::on_writer_matched(RTPSWriter*, const MatchingInfo&)``
   * - ``WriterListener::onWriterChangeReceivedByAll(RTPSWriter*, CacheChange_t*)``
     - ``WriterListener::on_writer_change_received_by_all(RTPSWriter*, CacheChange_t*)``
   * - ``TypeLookupReplyListener::onWriterChangeReceivedByAll(RTPSWriter*, CacheChange_t*)``
     - ``TypeLookupReplyListener::on_writer_change_received_by_all(RTPSWriter*, CacheChange_t*)``
   * - ``RTPSReader::getListener()``
     - ``RTPSReader::get_listener()``
   * - ``RTPSReader::setListener()``
     - ``RTPSReader::set_listener()``
   * - ``RTPSReader::expectsInlineQos()``
     - ``RTPSReader::expects_inline_qos()``
   * - ``RTPSReader::isInCleanState()``
     - ``RTPSReader::is_in_clean_state()``
   * - ``RTPSReader::getHistory()``
     - ``RTPSReader::get_history()``
   * - ``RTPSReader::nextUnreadCache(CacheChange_t**, WriterProxy**)``
     - ``RTPSReader::next_unread_cache()``
   * - ``RTPSReader::nextUntakenCache(CacheChange_t**, WriterProxy**)``
     - ``RTPSReader::next_untaken_cache()``
   * - ``ReaderListener::onReaderMatched(RTPSReader*, MatchingInfo&)``
     - ``ReaderListener::on_reader_matched(RTPSReader*, MatchingInfo&)``
   * - ``ReaderListener::onNewCacheChangeAdded(RTPSReader*, const CacheChange_t* const)``
     - ``ReaderListener::on_new_cache_change_added(RTPSReader*, const CacheChange_t* const)``
   * - ``TopicDataType::getSerializedSizeProvider(const void* const, DataRepresentationId_t)``
     - ``TopicDataType::calculate_serialized_size(const void* const, DataRepresentationId_t)``
   * - ``TopicDataType::createData()``
     - ``TopicDataType::create_data()``
   * - ``TopicDataType::deleteData(void*)``
     - ``TopicDataType::delete_data(void*)``
   * - ``TopicDataType::getKey(const void* const, InstanceHand*, bool)``
     - ``TopicDataType::compute_key(const void* const, InstanceHand&, bool)``
   * - ``TopicDataType::setName(const char*)``
     - ``TopicDataType::set_name(const string&)``
   * - ``char* TopicDataType::getName()``
     - ``string& TopicDataType::get_name()``
   * - ``TypeSupport::calculate_serialized_size_provider(const void* const, DataRepresentationId_t)``
     - ``TypeSupport::calculate_serialized_size(const void* const, DataRepresentationId_t)``
   * - ``get_key(void, InstanceHandle_t*, bool)``
     - ``compute_key(SerializedPayload_t&, InstanceHandle_t&, bool)``
   * - ``DynamicPubSubType::createData()``
     - ``DynamicPubSubType::create_data()``
   * - ``DynamicPubSubType::deleteData(void*)``
     - ``DynamicPubSubType::delete_data(void*)``
   * - ``DynamicPubSubType::getKey(const void* const, InstanceHand*, bool)``
     - ``DynamicPubSubType::compute_key(const void* const, InstanceHand&, bool)``
   * - ``DynamicPubSubType::getSerializedSizeProvider(const void* const, DataRepresentationId_t)``
     - ``DynamicPubSubType::calculate_serialized_size(const void* const, DataRepresentationId_t)``

Review your code for any APIs marked with the ``FASTDDS_DEPRECATED`` and ``FASTDDS_TODO_BEFORE`` macros.
Note that these deprecated APIs have been removed in Fast DDS v3.
Make the necessary updates to your implementation to ensure compatibility with the new version.

.. _step-8-update-structs-enums-and-variables:

Step 8: Update structs, enums, and variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As part of the Fast DDS migration, several structs, enums, and variables have been updated. You will need to modify
your code to reflect these changes:

1. Enum and Variable Changes:

    * Rename ``DiscoveryProtocol_t`` to ``DiscoveryProtocol``.
    * Rename ``initialHeartbeatDelay`` to ``initial_heartbeat_delay``.
    * Rename ``heartbeatPeriod`` to ``heartbeat_period``.
    * Rename ``nackResponseDelay`` to ``nack_response_delay``.
    * Rename ``nackSupressionDuration`` to ``nack_supression_duration``.
    * Rename ``heartbeatResponseDelay`` to ``heartbeat_response_delay``.
    * Rename ``initialAcknackDelay`` to ``initial_acknack_delay``.
    * Rename ``expectsInlineQos`` to ``expects_inline_qos``.
    * Rename ``m_typeSize`` to ``max_serialized_type_size``.
    * Rename ``m_isGetKeyDefined`` to ``is_compute_key_provided``.
    * Rename ``m_topicDataTypeName`` to ``topic_data_typename``.

2. Extend Built-in Topics:

    * ``SubscriptionBuiltinTopicData`` has been extended with additional fields to mimic those of ``ReaderProxyData``.
    * ``PublicationBuiltinTopicData`` has been extended with additional fields to mimic those of ``WriterProxyData``.
    * ``ParticipantBuiltinTopicData`` has been extended to include the product version and fields from
      ``ParticipantProxyData``.

3. Other Struct Changes:

    * ``SendBuffersAllocationAttributes`` has a new attribute to define the allocation configuration of the
      ``NetworkBuffers``.
    * ``TypeConsistencyQos`` has been removed from ``DataReader``, and the ``TypeConsistencyEnforcementQosPolicy`` and
      ``DataRepresentationQosPolicy`` have been added.

.. _step-9-refactor-examples:

Step 9: Examples refactor
^^^^^^^^^^^^^^^^^^^^^^^^^

All examples in the Fast DDS project have been refactored to follow a consistent structure, having renamed files,
restructured classes, and updated the overall format. Additionally, some examples have been removed, renamed, combined
or had significant changes to their options and configurations. If you have integrated any example into your
own implementation, carefully review the updated examples to ensure compatibility with your project. As reference,
consider the example `Configuration <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/configuration>`_.
