<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-dds-layer-questions"></a>

<a id="dds-layer-frequently-asked-questions"></a>

# DDS LAYER Frequently Asked Questions

<a id="core"></a>

## CORE

<a id="entity"></a>

### Entity

**What is the significance of a unique ID in the context of DDS and RTPS entities, and why is it important to have a shared ID between these entities?**

<br/>  The unique ID ensures that each entity within the distributed system can be distinctly identified.
This is crucial for maintaining the integrity and consistency of communications and operations within the system.
By having a shared ID between DDS (Data Distribution Service) and RTPS (Real-Time Publish-Subscribe) entities, the system can seamlessly map and correlate data across different communication protocols and frameworks.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**How does the shared ID between DDS and RTPS entities facilitate communication and interoperability within a distributed system?**

<br/>  A shared ID between DDS and RTPS entities enables a unified reference point for entities involved in communication.
This facilitates interoperability by allowing different components and services within the distributed system to easily recognize and interact with each other based on their unique identifiers, thus simplifying the integration and coordination processes.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**In what ways does storing the ID on an Instance Handle object and declaring it on the Entity base class adhere to principles of object-oriented design, such as encapsulation and inheritance?**

<br/>  Storing the ID on an Instance Handle object and declaring it on the [`Entity`](../../api_reference/dds_pim/core/entity.md#_CPPv4N8eprosima7fastdds3dds6EntityE) base class adheres to encapsulation by keeping the ID management within a dedicated object.
This design also leverages inheritance, allowing derived classes to inherit the ID-related functionality from the base class, promoting code reuse and reducing redundancy.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**How might the ability to access the unique ID via the "get_instance_handle()" function influence the implementation and maintenance of a distributed system?**

<br/>  The ability to access the unique ID via the `get_instance_handle()` function simplifies the implementation and maintenance of the system.
Developers can uniformly manage entity identification across various parts of the system, making it easier to track, debug, and update entity-related code.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**What are the advantages and potential drawbacks of using listeners for asynchronous notifications in a DDS system, and how can these drawbacks be mitigated?**

<br/>  Listeners provide real-time notifications of status changes, improving responsiveness and allowing for event-driven programming.
However, drawbacks include the potential for increased complexity and resource contention. Mitigation strategies involve keeping listener functions simple and offloading heavy processing to other parts of the system.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**How does the inheritance of listener interfaces across different entity types enhance the flexibility and modularity of the system?**

<br/>  The inheritance of listener interfaces enhances flexibility by allowing different entity types to share common callback mechanisms while enabling customization for specific types.
This modularity simplifies code management and fosters reuse.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**How does the limitation of operations on disabled entities influence the design and implementation of a DDS-based system?**

<br/>  Disabled entities can only perform basic operations such as QoS and listener management, status querying, and subentity creation/deletion.
This restriction ensures that incomplete or improperly configured entities do not adversely impact the system.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**What potential issues could arise from creating or deleting entities within a listener callback, and why is it recommended to avoid such actions?**

<br/>  Creating or deleting entities within listener callbacks can cause race conditions, deadlocks, or undefined behavior due to concurrent access.
It is recommended to use listeners solely for event notification and delegate entity management to higher-level components outside the callback scope.
For further information refer to [Listener](../../dds_layer/core/entity/entity.md#dds-layer-core-entity-commonchars-listener).

---

**In what ways do these custom callbacks differ from standard DDS callbacks, and what additional capabilities do they provide to the application developers?**

<br/> Unlike standard DDS callbacks, Fast DDS custom callbacks are always enabled and offer functionality tailored to the Fast DDS implementation.
They provide more granular control over participant and data discovery processes, enhancing the application’s ability to react to dynamic changes.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

<a id="policy"></a>

### Policy

**How do QoS policies influence the behavior of DDS entities, and what are the potential impacts of misconfiguring these policies on system performance and reliability?**

<br/>  QoS policies determine the operational parameters of DDS entities, such as latency, reliability, and resource usage.
Misconfigured QoS policies can lead to suboptimal performance, such as increased latency, dropped messages, or excessive resource consumption, which can negatively affect the overall system reliability and efficiency.
For further information refer to [Policy](../../dds_layer/core/policy/policy.md#dds-layer-core-policy).

---

**In what scenarios might it be necessary to modify the QoS policies of an entity after its creation, and what are the best practices for doing so using the "set_qos()" function?**

<br/>  Scenarios necessitating QoS modification post-creation include changes in network conditions, evolving application requirements, or the need to optimize performance.
Best practices include using the `set_qos()` function judiciously, validating the new policies before applying them, and monitoring the system for any adverse effects after changes.
For further information refer to [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).

---

**What are the implications of creating DDS entities in an enabled or disabled state, and how does the EntityFactoryQosPolicy affect this process?**

<br/>  Creating entities in an enabled state allows for immediate operation, while disabled entities require explicit enabling before full functionality.
The [`EntityFactoryQosPolicy`](../../api_reference/dds_pim/core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) governs this behavior, affecting initial system configuration and operational readiness.
For further information refer to [EntityFactoryQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#entityfactoryqospolicy).

---

**How do the specific callbacks provided by Fast DDS, such as on_participant_discovery and on_data_writer_discovery, enhance the functionality of the DDS system?**

<br/>  Fast DDS-specific callbacks, such as `on_participant_discovery` and `on_data_writer_discovery`, provide additional hooks for monitoring and responding to specific events within the DDS framework.
These callbacks offer greater control and insight into the system’s operational state.
For further information refer to [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity).

---

**How does the DeadlineQoS policy apply differently to topics with keys compared to those without keys, and what are the practical considerations for using keys in such scenarios?**

<br/>  For topics with keys, the [`DeadlineQosPolicy`](../../api_reference/dds_pim/core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) is applied individually to each key.
This means that each unique key (e.g., each vehicle in a fleet) must meet its deadline.
The practical consideration is that the publisher must manage deadlines for multiple keys simultaneously, which can be complex but allows for more granular control over data timeliness.
For further information refer to [Standard QoS Policies](../../dds_layer/core/policy/standardQosPolicies.md#standard).

---

**Why is it crucial for the offered deadline period on DataWriters to be less than or equal to the requested deadline period on DataReaders, and what could be the consequences of mismatched periods?**

<br/>  The requirement for the offered deadline period on DataWriters to be less than or equal to the requested deadline period on DataReaders ensures that the DataWriter can meet the DataReader’s expectations.
Mismatched periods could result in the DataReader perceiving missed deadlines, leading to potential data loss and reliability issues.
For further information refer to [Compatibility Rule](../../dds_layer/core/policy/standardQosPolicies.md#deadline-compatibilityrule).

---

**Why is it crucial for the offered deadline period on DataWriters to be less than or equal to the requested deadline period on DataReaders, and what could be the consequences of mismatched periods?**

<br/>  The requirement for the offered deadline period on DataWriters to be less than or equal to the requested deadline period on DataReaders ensures that the DataWriter can meet the DataReader’s expectations.
Mismatched periods could result in the DataReader perceiving missed deadlines, leading to potential data loss and reliability issues.
For further information refer to [Compatibility Rule](../../dds_layer/core/policy/standardQosPolicies.md#deadline-compatibilityrule).

---

**How should the DeadlineQosPolicy be configured in conjunction with the TimeBasedFilterQosPolicy to ensure consistency and avoid data loss or delays?**

<br/>  To ensure consistency, the [`DeadlineQosPolicy`](../../api_reference/dds_pim/core/policy/deadlineqospolicy.md#_CPPv4N8eprosima7fastdds3dds17DeadlineQosPolicyE) period must be greater or equal to the minimum separation specified in the [`TimeBasedFilterQosPolicy`](../../api_reference/dds_pim/core/policy/timebasedfilterqospolicy.md#_CPPv4N8eprosima7fastdds3dds24TimeBasedFilterQosPolicyE).
This prevents the system from attempting to enforce a stricter deadline than the filter allows, avoiding unnecessary alarms and ensuring smooth data flow.
For further information refer to [TimeBasedFilterQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#timebasedfilterqospolicy).

---

**How does the default value of c_TimeInfinite for the period in DeadlineQoS affect the behavior of DataWriters and DataReaders, and under what circumstances might this default value be modified?**

<br/>  The default value of `c_TimeInfinite` means that there is no deadline, so DataWriters and DataReaders are not constrained by time.
This is useful for applications where timeliness is not critical.
However, for time-sensitive applications, this default should be changed to a specific duration to ensure timely data updates.
For further information refer to [DeadlineQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#deadlineqospolicy).

---

**What are the key differences between BY_RECEPTION_TIMESTAMP and BY_SOURCE_TIMESTAMP in DestinationOrderQoS, and how do these settings impact the consistency and order of received data?**

<br/>  `BY_RECEPTION_TIMESTAMP` orders data based on when it is received, which can lead to different DataReaders having different final values due to network delays.
`BY_SOURCE_TIMESTAMP` ensures consistency across all DataReaders by using the send time from the DataWriter.
`BY_SOURCE_TIMESTAMP` is preferred for ensuring consistent data states across multiple DataReaders.
Both can be configured by using [`DestinationOrderQosPolicyKind`](../../api_reference/dds_pim/core/policy/destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKindE).
For further information refer to [DestinationOrderQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#destinationorderqospolicy).

---

**In what scenarios might the BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS be preferred over BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS, and vice versa?**

<br/>  `BY_RECEPTION_TIMESTAMP` might be preferred in scenarios where the most recent data is always the most relevant, regardless of source time (e.g., real-time sensor data).
`BY_SOURCE_TIMESTAMP` is ideal for applications requiring consistency, such as financial transactions or coordinated control systems.
For further information refer to [DestinationOrderQosPolicyKind](../../dds_layer/core/policy/standardQosPolicies.md#destinationorderqospolicykind).

---

**What are the potential challenges and solutions when DataWriters and DataReaders have incompatible DestinationOrderQoS kinds, and how does the compatibility rule ensure proper data ordering?**

<br/>  Incompatible kinds can lead to data being ignored or reordered incorrectly, causing inconsistencies.
The compatibility rule ensures that the [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) can handle the ordering provided by the [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter).
Solutions include aligning QoS settings across DataWriters and DataReaders and using appropriate fallback mechanisms.
For further information refer to [Compatibility Rule](../../dds_layer/core/policy/standardQosPolicies.md#destinationorder-compatibilityrule).

---

**How might the DestinationOrderQoS policy be applied in a multi-DataWriter scenario to ensure data consistency, and what are the potential pitfalls to avoid?**

<br/>  In scenarios with multiple DataWriters, such as collaborative robotics or distributed simulations, the [`DestinationOrderQosPolicy`](../../api_reference/dds_pim/core/policy/destinationorderqospolicy.md#_CPPv4N8eprosima7fastdds3dds25DestinationOrderQosPolicyE) ensures that data from different writers is correctly ordered.
Avoiding pitfalls like network-induced delays involves carefully configuring timestamps and ensuring synchronized clocks across systems.
For further information refer to [DestinationOrderQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#destinationorderqospolicy).

---

<a id="status"></a>

### Status

**What role do status objects play in the communication lifecycle of DDS entities, and how do they interact with listener callbacks to notify applications of status changes?**

<br/>  Status objects track the communication state of entities, triggering listener callbacks when changes occur.
This mechanism ensures that applications are promptly informed of relevant status updates, facilitating timely responses to communication events.
For further information refer to [Status](../../dds_layer/core/status/status.md#dds-layer-core-status).

---

**How does the concept of StatusCondition link entities to Wait-sets, and what benefits does this linkage provide in terms of system synchronization and event handling?**

<br/> `StatusCondition` provides a means to monitor multiple status changes efficiently by linking entities to Wait-sets.
This linkage allows for consolidated event handling, reducing polling overhead and improving synchronization within the system.
For further information refer to [Status](../../dds_layer/core/status/status.md#dds-layer-core-status).

---

**How does the use of StatusMask in enabling or disabling specific callbacks affect the responsiveness and behavior of a DDS system?**

<br/> The [`StatusMask`](../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) allows selective enabling or disabling of specific callbacks, fine-tuning the system’s responsiveness and avoiding unnecessary processing.
Proper management of [`StatusMask`](../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) settings ensures that only relevant events trigger callbacks, optimizing system behavior.
For further information refer to [Status](../../dds_layer/core/status/status.md#dds-layer-core-status).

---

<a id="domain"></a>

## DOMAIN

**Which are the mandatory arguments to create a DomainParticipant?**

<br/> The mandatory arguments to create a [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) are the `DomainId` that identifies the domain where the DomainParticipant will be created and the [`DomainParticipantQos`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) describing the behavior of the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
If the provided value is `TOPIC_QOS_DEFAULT`, the value of the DomainParticipantQos is used.
For further information refer to [Creating a DomainParticipant](../../dds_layer/domain/domainParticipant/createDomainParticipant.md#dds-layer-domainparticipant-creation).

---

**What is the purpose of providing a "DomainId" when creating a DomainParticipant?**

<br/> The `DomainId` identifies the domain where the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) will be created.
Do not use `DomainId` higher than 200.
Once created the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), its `DomainId` can not be changed.
For further information refer to [Domain](../../dds_layer/domain/domain.md#dds-layer-domain).

---

**What is the purpose of providing a Listener when creating a DomainParticipant?**

<br/> A Listener derived from [`DomainParticipantListener`](../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE), implementing the callbacks that will be triggered in response to events and state changes on the DomainParticipant.
By default, empty callbacks are used.
For further information about [`DomainParticipantListener`](../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) and how to implement its callbacks refer to [DomainParticipantListener](../../dds_layer/domain/domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener).

---

**What is the purpose of providing a "StatusMask" when creating a DomainParticipant?**

<br/> A [`StatusMask`](../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the [`DomainParticipantListener`](../../api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE).
By default, all events are enabled.
For further information refer to [DomainParticipantListener](../../dds_layer/domain/domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener).

---

**What happens when "create_participant_with_profile()" returns a null pointer in creating a DomainParticipant?**

<br/> `create_participant_with_profile()` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.
For further information refer to [Creating a DomainParticipant](../../dds_layer/domain/domainParticipant/createDomainParticipant.md#dds-layer-domainparticipant-creation).

---

**What is the primary way to create a Publisher in the context of DomainParticipant?**

<br/> A [`Publisher`](../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) always belongs to a [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
Creation of a Publisher is done with the `create_publisher()` member function on the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) instance.
For further information refer to [Creating a Publisher](../../dds_layer/publisher/publisher/createPublisher.md#dds-layer-publisher-creation).

---

**What settings are used in the creation of a DomainParticipant?**

<br/> If there is an XML profile exported in the environment, those settings will be used.
If the profile has not been exported, the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) will be created with the default values per [`DomainParticipantQos`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) and `0` as `DomainId`.
For further information refer to [Default profile DomainParticipant creation](../../dds_layer/domain/domainParticipant/createDomainParticipant.md#dds-layer-domainparticipant-creation-default-profile).

---

**Why am I unable to delete a DomainParticipant?**

<br/> To delete a [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), it is necessary to delete first all its associated entities to this [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
Otherwise, the function will issue an error and the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) will not be deleted.
To delete the entities, the `delete_contained_entities` member function must be used, or the entities must be deleted individually.
For further information refer to [Deleting a DomainParticipant](../../dds_layer/domain/domainParticipant/createDomainParticipant.md#dds-layer-domainparticipant-deletion).

---

**How do I modify the behavior of a DomainParticipant?**

<br/> To modify the behavior of a [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), the QoS values specified in the [`DomainParticipantQos`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE).
These QoS values can be set at the creation of the DomainParticipant or modified with the [`DomainParticipant::set_qos()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos) member function.
For further information refer to [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).

---

**Under what circumstances does calling "ignore_participant()" member of a DomainParticipant cause deadlock?**

<br/> When calling `ignore_participant` inside the listener.
This should be used only when there is a need to ignore participants inside the discovery callback.
For further information refer to [DomainParticipantListener](../../dds_layer/domain/domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener).

---

<a id="publisher"></a>

## PUBLISHER

**What is the purpose of providing a PublisherQos when creating a Publisher?**

<br/> The PublisherQos describes the behavior of the Publisher.
If the provided value is [`PUBLISHER_QOS_DEFAULT`](../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds21PUBLISHER_QOS_DEFAULTE), the value of the default PublisherQos is used.
For further information refer to [PublisherQos](../../dds_layer/publisher/publisher/publisher.md#dds-layer-publisher-publisherqos).

---

**What is the purpose of providing a Listener when creating a Publisher?**

<br/> A Listener derived from [`PublisherListener`](../../api_reference/dds_pim/publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE), implementing the callbacks that will be triggered in response to events and state changes on the Publisher.
By default, empty callbacks are used.
For further information refer to [PublisherListener](../../dds_layer/publisher/publisherListener/publisherListener.md#dds-layer-publisher-publisherlistener).

---

**What happens when "create_publisher()" returns a null pointer during the creation of a Publisher?**

<br/> `create_publisher()` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.
This also applies to Subscriber and [`Topic`](../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE).
For further information refer to [Creating a Publisher](../../dds_layer/publisher/publisher/createPublisher.md#dds-layer-publisher-creation).

---

**What is the purpose of the "StatusMask" in creating a Publisher or a Subscriber or a Topic?**

<br/> A [`StatusMask`](../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the Publisher/Subscriber/Topic listener.
By default, all events are enabled.
For further information refer to [Creating a Publisher](../../dds_layer/publisher/publisher/createPublisher.md#dds-layer-publisher-creation).

---

**What happens when "create_publisher_with_profile()" returns a null pointer?**

<br/> `create_publisher_with_profile()` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.
This is also valid for Subscribers.
For further information refer to [Profile based creation of a Publisher](../../dds_layer/publisher/publisher/createPublisher.md#dds-layer-publisher-creation-profile).

---

**What happens when "create_datawriter_with_profile()" returns a null pointer in creating a DataWriter?**

<br/> `create_datawriter_with_profile()` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.
This is also valid for DataReaders.
For further information refer to [Profile based creation of a DataWriter](../../dds_layer/publisher/dataWriter/createDataWriter.md#dds-layer-publisher-datawriter-creation-profile).

---

**What is an efficient way to write large data types in DDS using a DataWriter to minimize memory usage and processing time?**

<br/> An efficient way to write large data types in DDS is to make the [`DataWriter`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) loan a sample from its memory to the user, and the user to fill this sample with the required values.
When [`write()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) is called with such a loaned sample, the [`DataWriter`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) does not copy its contents, as it already owns the buffer.
For further information refer to [Borrowing a data buffer](../../dds_layer/publisher/dataWriter/publishingData.md#dds-layer-publisher-write-loans).

---

**What happens to the contents of a loaned data sample after "write()" has been successfully called with that sample?**

<br/> Once [`write()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) has been called with a loaned sample, the loan is considered returned, and it is not safe to make any changes on the contents of the sample.
For further information refer to [Borrowing a data buffer](../../dds_layer/publisher/dataWriter/publishingData.md#dds-layer-publisher-write-loans).

---

**What happens if a loaned data sample is not written by the user?**

<br/> If function [`loan_sample()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind) is called but the sample is never written, the loan must be returned to the DataWriter using [`discard_loan()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter12discard_loanERPv). Otherwise, the DataWriter may run out of samples.
For further information refer to [Borrowing a data buffer](../../dds_layer/publisher/dataWriter/publishingData.md#dds-layer-publisher-write-loans).

---

**Why am I unable to delete a Publisher?**

<br/> To delete a Publisher, it is necessary to delete first all its associated entities.
Otherwise, the function will issue an error and the Publisher will not be deleted.
To delete the entities, the [`delete_contained_entities()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader25delete_contained_entitiesEv) member function must be used.
This is also valid for Subscribers.
For further information refer to [Deleting a Publisher](../../dds_layer/publisher/publisher/createPublisher.md#dds-layer-publisher-deletion).

---

**What is the meaning of the value "DATAWRITER_QOS_DEFAULT"?**

<br/> The value [`DATAWRITER_QOS_DEFAULT`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds22DATAWRITER_QOS_DEFAULTE) has different meanings depending on where it is used.
This is also applicable for the [`DATAWRITER_QOS_DEFAULT`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds22DATAWRITER_QOS_DEFAULTE) and [`TOPIC_QOS_DEFAULT`](../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds17TOPIC_QOS_DEFAULTE).
For further information refer to [DataWriterQos](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos).
\* On `create_datawriter()` and [`DataWriter::set_qos()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter7set_qosERK13DataWriterQos) it refers to the default DataWriterQos as returned by `get_default_datawriter_qos()`.
\* On `set_default_datawriter_qos()` it refers to the default constructed [`DataWriterQos()`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos13DataWriterQosEv).

---

**When could a sample be removed without being received by the DataReaders?**

<br/> This could happen in constrained networks or if the publication throughput is too demanding.
The callback `on_unacknowledged_sample_removed()` can be used to detect these situations so the publishing application can apply some solution to ease this issue like reducing the publication rate.
For further information refer to [on_unacknowledged_sample_removed callback](../../dds_layer/publisher/dataWriterListener/dataWriterListener.md#dds-layer-publisher-datawriterlistener-on-unack-sample-removed).

---

<a id="subscriber"></a>

## SUBSCRIBER

**What is the primary consideration when accessing elements on a DDS data sequence after calling "DataReader::read()" or "DataReader::take()" operations?**

<br/> After calling the [`DataReader::read()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4readER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) or [`DataReader::take()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4takeER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) operations, accessing the data on the returned sequences is quite easy.
The sequences API provides a `length()` operation returning the number of elements in the collections.
The application code just needs to check this value and use the `[]` operator to access the corresponding elements.
Elements on the DDS data sequence should only be accessed when the corresponding element on the SampleInfo sequence indicates that valid data is present.
When using Data Sharing, it is also important to check that the sample is valid (i.e. not replaced, refer to DataReader and DataWriter history coupling for further information in this regard.
For further information refer to [Processing returned data](../../dds_layer/subscriber/dataReader/readingData.md#dds-layer-subscriber-accessreceived-data).

---

**What is the primary method for an application to receive new data values from a DataReader without relying on a Listener?**

<br/> Instead of relying on the Listener to try and get new data values, the application can also dedicate a thread to wait until any new data is available on the [`DataReader`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE).
This can be done using a wait-set to wait for a change on the DataAvailable status.
For further information refer to [Accessing data with a waiting thread](../../dds_layer/subscriber/dataReader/readingData.md#dds-layer-subscriber-accessreceived-wait).

---

**How do the "DataReader::read()" and "DataReader::take()" operations (and their variants) return information to the application?**

<br/> These two operations return information in two sequences: the received DDS data samples are returned in a sequence of the data type and the corresponding information about each DDS sample is returned in a SampleInfo sequence.
For further information refer to [Loaning and Returning Data and SampleInfo Sequences](../../dds_layer/subscriber/dataReader/readingData.md#dds-layer-subscriber-accessreceived-loans).

---

**What is the consequence if an application does not return loaned sequences back to the middleware?**

<br/> If the application does not return the loan by calling the [`DataReader::return_loan()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader11return_loanER18LoanableCollectionR13SampleInfoSeq) operation, then Fast DDS will eventually run out of memory to store DDS data samples received from the network for that DataReader.
For further information refer to [Loaning and Returning Data and SampleInfo Sequences](../../dds_layer/subscriber/dataReader/readingData.md#dds-layer-subscriber-accessreceived-loans).

---

**What is the purpose of "disposed_generation_count" in relation to an object's lifecycle?**

<br/> `disposed_generation_count` indicates the number of times the instance had become alive after it was disposed.
For further information refer to [disposed_generation_count](../../dds_layer/subscriber/sampleInfo/sampleInfo.md#dds-layer-subscriber-sampleinfo-disposedgenerationcount).

---

**What is the purpose of "no_writers_generation_count" in relation to the instance's lifecycle?**

<br/> `no_writers_generation_count` indicates the number of times the instance had become alive after it was disposed as `NOT_ALIVE_NO_WRITERS`.
For further information refer to [no_writers_generation_count](../../dds_layer/subscriber/sampleInfo/sampleInfo.md#dds-layer-subscriber-sampleinfo-nowritersgenerationcount).

---

**What is the purpose of the "related_sample_identity" extension in a requester-replier configuration?**

<br/> `related_sample_identity` is an extension for requester-replier configuration.
On reply messages, it contains the sample_identity of the related request message.
It is used by the requester to be able to link each reply to the appropriate request.
For further information refer to [related_sample_identity](../../dds_layer/subscriber/sampleInfo/sampleInfo.md#dds-layer-subscriber-sampleinfo-relatedsampleidentity).

---

**What is the purpose of setting "valid_data" to false in a data sample?**

<br/> `valid_data` is a boolean that indicates whether the data sample contains a change in the value or not.
Samples with this value set to false are used to communicate a change in the instance status, e.g., a change in the liveliness of the instance.
In this case, the data sample should be dismissed as all the relevant information is in the data members of SampleInfo.
For further information refer to [valid_data](../../dds_layer/subscriber/sampleInfo/sampleInfo.md#dds-layer-subscriber-sampleinfo-validdata).

---

<a id="topic"></a>

## TOPIC

**What is the primary function of a Topic in the context of Publisher-Subscriber communication?**

<br/> A [`Topic`](../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) is a specialization of the broader concept of TopicDescription.
A [`Topic`](../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) represents a single data flow between Publisher and Subscriber.
For further information refer to [Topic](../../dds_layer/topic/topic.md#dds-layer-topic).

---

**What is the difference between Topics, Keys and Instances?**

<br/> Topics define the structure of the data being exchanged and are linked to a specific data type.
Keys are fields within the data type that uniquely identify different instances of the same Topic.
An Instance refers to a specific set of data distinguished by its key values within a Topic, allowing multiple sets of related data (instances) to exist under a single Topic, with updates being directed to the correct instance based on the key.
For further information refer to [Topics, keys and instances](../../dds_layer/topic/instances.md#dds-layer-topic-instances).

---

**How is a Topic created?**

<br/> Creation of a Topic is done with the [`create_topic()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12create_topicERKNSt6stringERKNSt6stringERK8TopicQosR12ReturnCode_tP13TopicListenerRK10StatusMask) member function on the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) instance, that acts as a factory for the Topic.
Mandatory arguments for creating a Topic a string with the name that identifies the topic, the name of the registered data type that will be transmitted and the [`TopicQos`](../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds8TopicQosE) that describes the behavior of the Topic.
For further information refer to [Creating a Topic](../../dds_layer/topic/topic/createTopic.md#dds-layer-topic-creation).

---

**What type of values can the "like" operator be used with in the context of content Filtered Topic?**

<br/> The like operator is similar to the one defined by SQL.
This operator can only be used with strings. There are two wildcards that could be used in conjunction with this operator.
For further information refer to [The default SQL-like filter](../../dds_layer/topic/contentFilteredTopic/defaultFilter.md#dds-layer-topic-contentfilteredtopic-default-filter).

---

**What does the "like" operator represent in terms of character matching?**

<br/> The percent sign `%"``(or its alias ``*`) represents zero, one, or multiple characters.
For further information refer to [The default SQL-like filter](../../dds_layer/topic/contentFilteredTopic/defaultFilter.md#dds-layer-topic-contentfilteredtopic-default-filter).

---

**What does the "like" operator's wildcard represent when used in a string comparison?**

<br/> The underscore sign `_` (or its alias `?`) represents one single character.
For further information refer to [The default SQL-like filter](../../dds_layer/topic/contentFilteredTopic/defaultFilter.md#dds-layer-topic-contentfilteredtopic-default-filter).

---

**How is filtering disabled?**

<br/> If the expression is an empty string, that disables filtering as explained in creating a ContentFilteredTopic.
For further information refer to [Filtering data on a Topic](../../dds_layer/topic/contentFilteredTopic/createContentFilteredTopic.md#dds-layer-topic-filtering-data-on-topic).

---

**How to enable the usage of a Custom Filter in an application?**

<br/> To be able to use the Custom Filter in an application, the Custom Filter’s factory must be registered in the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
The next snippet code shows how to register a factory through the API function `register_content_filter_factory()`.
For further information refer to [Registering the Factory](../../dds_layer/topic/contentFilteredTopic/customFilters.md#dds-layer-topic-customfilter-register).

---

**What will happen if an error occurs during the creation of a ContentFilteredTopic using the "create_contentfilteredtopic()" function?**

<br/> `create_contentfilteredtopic()` will return a null pointer if there was an error during the operation, e.g., if the related Topic belongs to a different DomainParticipant, a Topic with the same name already exists, syntax errors on the filter expression, or missing parameter values.
It is advisable to check that the returned value is a valid pointer.
For further information refer to [Creating a ContentFilteredTopic using the Custom Filter](../../dds_layer/topic/contentFilteredTopic/customFilters.md#dds-layer-topic-customfilter-create-topic).

---

**What is the method used to delete a ContentFilteredTopic in the context of a DomainParticipant?**

<br/> A ContentFilteredTopic can be deleted with the `delete_contentfilteredtopic()` member function on the DomainParticipant instance where the ContentFilteredTopic was created.
For further information refer to [Deleting a ContentFilteredTopic](../../dds_layer/topic/contentFilteredTopic/createContentFilteredTopic.md#dds-layer-topic-contentfilteredtopic-deletion).

<br/>
