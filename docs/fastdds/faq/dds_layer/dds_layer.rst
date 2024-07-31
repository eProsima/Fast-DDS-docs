.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _freq_dds_layer_questions:

DDS LAYER Frequently Asked Questions
====================================

CORE
----

.. collapse::  What is the significance of a unique ID in the context of DDS and RTPS entities, and why is it important to have a shared ID between these entities?




    :Answer:

    The unique ID ensures that each entity within the distributed system can be distinctly identified. This is crucial for maintaining the integrity and consistency of communications and operations within the system. By having a shared ID between DDS (Data Distribution Service) and RTPS (Real-Time Publish-Subscribe) entities, the system can seamlessly map and correlate data across different communication protocols and frameworks.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the shared ID between DDS and RTPS entities facilitate communication and interoperability within a distributed system?




    :Answer:

    A shared ID between DDS and RTPS entities enables a unified reference point for entities involved in communication. This facilitates interoperability by allowing different components and services within the distributed system to easily recognize and interact with each other based on their unique identifiers, thus simplifying the integration and coordination processes.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  In what ways does storing the ID on an Instance Handle object and declaring it on the Entity base class adhere to principles of object-oriented design, such as encapsulation and inheritance?




    :Answer:

    Storing the ID on an **Instance Handle** object and declaring it on the **Entity** base class adheres to encapsulation by keeping the ID management within a dedicated object. This design also leverages inheritance, allowing derived classes to inherit the ID-related functionality from the base class, promoting code reuse and reducing redundancy.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How might the ability to access the unique ID via the ``get_instance_handle()`` function influence the implementation and maintenance of a distributed system?




    :Answer:

    The ability to access the unique ID via the ``get_instance_handle()`` function simplifies the implementation and maintenance of the system. Developers can uniformly manage entity identification across various parts of the system, making it easier to track, debug, and update entity-related code.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What potential security concerns could arise from having a shared ID between DDS and RTPS entities, and how can these concerns be mitigated?




    :Answer:

    Potential security concerns include unauthorized access or manipulation of the unique IDs. These can be mitigated by implementing access controls, authentication mechanisms, and encryption to protect the IDs and the Instance Handle objects from unauthorized access and tampering.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What error handling mechanisms should be in place when accessing the Instance Handle object through the ``get_instance_handle()`` function to ensure system robustness?




    :Answer:

    Error handling mechanisms should include checks for null or invalid Instance Handle objects, exceptions for failed retrievals, and logging of errors for diagnostic purposes. Additionally, implementing retries and fallback procedures can help maintain system robustness in case of transient errors.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How might the storage and retrieval of unique IDs via Instance Handle objects impact the performance of a distributed system, particularly in high-throughput or real-time scenarios?




    :Answer:

    The impact on performance depends on the efficiency of the storage and retrieval mechanisms for Instance Handle objects. Optimized data structures and caching strategies can minimize latency and overhead, ensuring that the system remains performant even under high-throughput or real-time conditions.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  In what ways does the design choice of using a unique ID for each entity affect the scalability of the system as the number of entities grows?




    :Answer:

    Using unique IDs for each entity supports scalability by allowing the system to manage a large number of entities without confusion or collision. As the number of entities grows, the system can efficiently track and manage each one through its unique ID, ensuring consistent performance and reliability.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the use of Instance Handle objects and the ``get_instance_handle()`` function support or hinder the integration of new components or legacy systems within the distributed architecture?




    :Answer:

    The design choice of using Instance Handle objects and the ``get_instance_handle()`` function enhances integration by providing a standardized way to identify and access entities. This standardization simplifies the addition of new components and the integration of legacy systems, as all parts of the system can rely on a consistent entity identification method.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How do QoS policies influence the behavior of DDS entities, and what are the potential impacts of misconfiguring these policies on system performance and reliability?




    :Answer:

    QoS policies determine the operational parameters of DDS entities, such as latency, reliability, and resource usage. Misconfigured QoS policies can lead to suboptimal performance, such as increased latency, dropped messages, or excessive resource consumption, which can negatively affect the overall system reliability and efficiency.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  In what scenarios might it be necessary to modify the QoS policies of an entity after its creation, and what are the best practices for doing so using the ``set_qos()`` function?




    :Answer:

    Scenarios necessitating QoS modification post-creation include changes in network conditions, evolving application requirements, or the need to optimize performance. Best practices include using the ``set_qos()`` function judiciously, validating the new policies before applying them, and monitoring the system for any adverse effects after changes.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the advantages and potential drawbacks of using listeners for asynchronous notifications in a DDS system, and how can these drawbacks be mitigated?




    :Answer:

    Listeners provide real-time notifications of status changes, improving responsiveness and allowing for event-driven programming. However, drawbacks include the potential for increased complexity and resource contention. Mitigation strategies involve keeping listener functions simple and offloading heavy processing to other parts of the system.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the inheritance of listener interfaces across different entity types enhance the flexibility and modularity of the system?




    :Answer:

    The inheritance of listener interfaces enhances flexibility by allowing different entity types to share common callback mechanisms while enabling customization for specific types. This modularity simplifies code management and fosters reuse.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What role do status objects play in the communication lifecycle of DDS entities, and how do they interact with listener callbacks to notify applications of status changes?




    :Answer:

    Status objects track the communication state of entities, triggering listener callbacks when changes occur. This mechanism ensures that applications are promptly informed of relevant status updates, facilitating timely responses to communication events.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the concept of StatusCondition link entities to Wait-sets, and what benefits does this linkage provide in terms of system synchronization and event handling?




    :Answer:

    **StatusCondition** provides a means to monitor multiple status changes efficiently by linking entities to Wait-sets. This linkage allows for consolidated event handling, reducing polling overhead and improving synchronization within the system.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the implications of creating DDS entities in an enabled or disabled state, and how does the EntityFactoryQosPolicy affect this process?




    :Answer:

    Creating entities in an enabled state allows for immediate operation, while disabled entities require explicit enabling before full functionality. The **EntityFactoryQosPolicy** governs this behavior, affecting initial system configuration and operational readiness.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the limitation of operations on disabled entities influence the design and implementation of a DDS-based system?




    :Answer:

    Disabled entities can only perform basic operations such as QoS and listener management, status querying, and subentity creation/deletion. This restriction ensures that incomplete or improperly configured entities do not adversely impact the system.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the use of StatusMask in enabling or disabling specific callbacks affect the responsiveness and behavior of a DDS system?




    :Answer:

    The **StatusMask** allows selective enabling or disabling of specific callbacks, fine-tuning the system's responsiveness and avoiding unnecessary processing. Proper management of StatusMask settings ensures that only relevant events trigger callbacks, optimizing system behavior.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What strategies can be employed to manage the complexity and potential performance issues arising from having multiple listener callbacks, especially given the single-threaded nature of listener execution?




    :Answer:

    To manage complexity and performance, listener functions should be concise, delegating extensive processing to other components. Utilizing multi-threading or task queues can help distribute the workload, preventing bottlenecks and ensuring efficient execution.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What potential issues could arise from creating or deleting entities within a listener callback, and why is it recommended to avoid such actions?




    :Answer:

    Creating or deleting entities within listener callbacks can cause race conditions, deadlocks, or undefined behavior due to concurrent access. It is recommended to use listeners solely for event notification and delegate entity management to higher-level
    components outside the callback scope.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the best practices for implementing listener functions to ensure they remain efficient and do not lead to undefined behavior or performance bottlenecks?




    :Answer:

    Listener functions should be designed to quickly handle events and offload detailed processing. This approach minimizes blocking time within the listener thread, ensuring timely handling of subsequent events and maintaining system responsiveness.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How do the specific callbacks provided by Fast DDS, such as on_participant_discovery and on_data_writer_discovery, enhance the functionality of the DDS system?




    :Answer:

    Fast DDS-specific callbacks, such as ``on_participant_discovery `` and ``on_data_writer_discovery``, provide additional hooks for monitoring and responding to specific events within the DDS framework. These callbacks offer greater control and insight into the system's operational state.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  In what ways do these custom callbacks differ from standard DDS callbacks, and what additional capabilities do they provide to the application developers?




    :Answer:

    Unlike standard DDS callbacks, Fast DDS custom callbacks are always enabled and offer functionality tailored to the Fast DDS implementation. They provide more granular control over participant and data discovery processes, enhancing the application's ability to react to dynamic changes.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How important is thorough documentation for each entity type and its corresponding QoS policies and listener interfaces in ensuring effective system implementation and maintenance?




    :Answer:

    Comprehensive documentation is crucial for understanding the configuration options and behavior of different entity types, QoS policies, and listener interfaces. It aids developers in implementing and maintaining systems that align with expected functionality and performance.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the implications of setting a too short or too long deadline period on both the publishing and subscribing sides, and how does this affect the reliability and timeliness of data transmission?




    :Answer:

    Setting a too short deadline period on the publishing side can lead to frequent deadline misses if the application cannot keep up, causing unnecessary alarms and potential loss of trust in the system. On the subscribing side, a too short period can cause frequent notifications of missed deadlines, leading to potential data inconsistencies. Conversely, too long a deadline period might delay the detection of issues, affecting the timeliness and reliability of data transmission.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the DeadlineQoS policy apply differently to topics with keys compared to those without keys, and what are the practical considerations for using keys in such scenarios?




    :Answer:

    For topics with keys, the **DeadlineQoS policy** is applied individually to each key. This means that each unique key (e.g., each vehicle in a fleet) must meet its deadline. The practical consideration is that the publisher must manage deadlines for multiple keys simultaneously, which can be complex but allows for more granular control over data timeliness.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Why is it crucial for the offered deadline period on DataWriters to be less than or equal to the requested deadline period on DataReaders, and what could be the consequences of mismatched periods?




    :Answer:

    The requirement for the offered deadline period on **DataWriters** to be less than or equal to the requested deadline period on DataReaders ensures that the DataWriter can meet the DataReader's expectations. Mismatched periods could result in the DataReader perceiving missed deadlines, leading to potential data loss and reliability issues.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How should the DeadlineQoS policy be configured in conjunction with the TimeBasedFilterQoS policy to ensure consistency and avoid data loss or delays?




    :Answer:

    To ensure consistency, the **DeadlineQoS period** must be at least equal to the minimum separation specified in the **TimeBasedFilterQoS policy**. This prevents the system from attempting to enforce a stricter deadline than the filter allows, avoiding unnecessary alarms and ensuring smooth data flow.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How does the default value of c_TimeInfinite for the period in DeadlineQoS affect the behavior of DataWriters and DataReaders, and under what circumstances might this default value be modified?




    :Answer:

    The default value of ``c_TimeInfinite`` means that there is no deadline, so **DataWriters** and **DataReaders** are not constrained by time. This is useful for applications where timeliness is not critical. However, for time-sensitive applications, this default should be changed to a specific duration to ensure timely data updates.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the key differences between BY_RECEPTION_TIMESTAMP and BY_SOURCE_TIMESTAMP in DestinationOrderQoS, and how do these settings impact the consistency and order of received data?




    :Answer:

    ``BY_RECEPTION_TIMESTAMP`` orders data based on when it is received, which can lead to different DataReaders having different final values due to network delays. ``BY_SOURCE_TIMESTAMP`` ensures consistency across all **DataReaders** by using the send time from the **DataWriter**. ``BY_SOURCE_TIMESTAMP`` is preferred for ensuring consistent data states across multiple **DataReaders**.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  In what scenarios might the BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS be preferred over BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS, and vice versa?




    :Answer:

    ``BY_RECEPTION_TIMESTAMP`` might be preferred in scenarios where the most recent data is always the most relevant, regardless of source time (e.g., real-time sensor data). ``BY_SOURCE_TIMESTAMP`` is ideal for applications requiring consistency, such as financial transactions or coordinated control systems.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the potential challenges and solutions when DataWriters and DataReaders have incompatible DestinationOrderQoS kinds, and how does the compatibility rule ensure proper data ordering?




    :Answer:

    Incompatible kinds can lead to data being ignored or reordered incorrectly, causing inconsistencies. The compatibility rule ensures that the DataReader can handle the ordering provided by the DataWriter. Solutions include aligning QoS settings across DataWriters and DataReaders and using appropriate fallback mechanisms.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the best practices for handling errors or unexpected behavior when configuring and using DeadlineQoS and DestinationOrderQoS policies, especially in a dynamic or evolving system environment?




    :Answer:

    Best practices include thorough validation of QoS settings before deployment, using default values as fallbacks, and implementing robust error logging and alerting mechanisms. Regular audits and updates of QoS configurations help maintain optimal performance and prevent issues.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How might the DestinationOrderQoS policy be applied in a multi-DataWriter scenario to ensure data consistency, and what are the potential pitfalls to avoid?




    :Answer:

    In scenarios with multiple **DataWriters**, such as collaborative robotics or distributed simulations, the **DestinationOrderQoS** ensures that data from different writers is correctly ordered. Avoiding pitfalls like network-induced delays involves carefully configuring timestamps and ensuring synchronized clocks across systems.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What are the key considerations for users when creating and linking their own listeners to entities, and how can they ensure that their implementation aligns with the documented behavior and expected system responses?




    :Answer:

    When creating and linking listeners, users should ensure their implementation adheres to documented interfaces and expected behaviors. This includes proper handling of status changes, efficient callback execution, and alignment with the application's overall architecture and design principles.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

DOMAIN
------

.. collapse::  What is the purpose of providing a "DomainId" when creating a DomainParticipant?




    :Answer:

    The ``DomainId`` identifies the domain where the DomainParticipant will be created. Do not use ``DomainId`` higher than 200.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of providing a Listener when creating a DomainParticipant?




    :Answer:

    A **Listener** derived from **DomainParticipantListener**, implementing the callbacks that will be triggered in response to events and state changes on the **DomainParticipant**. By default, empty callbacks are used.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of providing a "StatusMask" when creating a DomainParticipant?




    :Answer:

    A ``StatusMask`` that activates or deactivates triggering of individual callbacks on the **DomainParticipantListener**. By default, all events are enabled.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens when "create_participant_with_profile()" returns a null pointer in creating a DomainParticipant?




    :Answer:

    ``create_participant_with_profile()`` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported. It is advisable to check that the returned value is a valid pointer.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary way to create a Publisher in the context of DomainParticipant?




    :Answer:

    A **Publisher** always belongs to a **DomainParticipant**. Creation of a Publisher is done with the ``create_publisher()`` member function on the DomainParticipant instance.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What settings are used in the creation of a DomainParticipant?




    :Answer:

    If there is an XML profile exported in the environment, those settings will be used. If the profile has not been exported, the **DomainParticipant** will be created with the default values per ``DomainParticipantQoS`` and ``0`` as ``DomainId``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Why am I unable to delete a DomainParticipant?




    :Answer:

    To delete a **DomainParticipant**, it is necessary to delete first all the **Entities** belonging to this **DomainParticipant**. Otherwise, the function will issue an error and the **DomainParticipant** will not be deleted. To delete the entities, the ``delete_contained_entities`` member function must be used, or the entities must be deleted individually.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How do I modify the behavior of a DomainParticipant?




    :Answer:

    To modify the behavior of a **DomainParticipant**, the QoS values specified in the **DomainParticipantQoS**. These QoS values can be set at the creation of the **DomainParticipant** or modified with the ``DomainParticipant::set_qos()`` member function.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Under what circumstances does calling "ignore_participant" cause deadlock?




    :Answer:

    When calling ``ignore_participant`` inside the listener. This should be used only when there is a need to ignore participants inside the discovery callback.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

PUBLISHER
---------

.. collapse::  What is the purpose of providing a PublisherQos when creating a Publisher?




    :Answer:

    The **PublisherQos** describes the behavior of the Publisher. If the provided value is ``PUBLISHER_QOS_DEFAULT``, the value of the Default **PublisherQos** is used.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of providing a Listener when creating a Publisher?




    :Answer:

    A **Listener** derived from **PublisherListener**, implementing the callbacks that will be triggered in response to events and state changes on the **Publisher**. By default, empty callbacks are used.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens when "create_publisher()" returns a null pointer during the creation of a Publisher?




    :Answer:

    ``create_publisher()`` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported. It is advisable to check that the returned value is a valid pointer. This also applies to Subscriber and Topic.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of the "StatusMask" in creating a Publisher or a Subscriber or a Topic?




    :Answer:

    A ``StatusMask`` that activates or deactivates triggering of individual callbacks on the PublisherListener/SubscriberListener/TopicListener. By default, all events are enabled.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens when "create_publisher_with_profile()" returns a null pointer?




    :Answer:

    ``create_publisher_with_profile()`` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported. It is advisable to check that the returned value is a valid pointer. This is also valid for Subscribers.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens when "create_datawriter_with_profile()" returns a null pointer in creating a DataWriter?




    :Answer:

    ``create_datawriter_with_profile()`` will return a null pointer if there was an error during the operation, e.g., if the provided QoS is not compatible or is not supported. It is advisable to check that the returned value is a valid pointer. This is also valid for DataReaders.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens to the contents of a loaned data sample after "write()" has been successfully called with that sample?




    :Answer:

    Once ``write()`` has been called with a loaned sample, the loan is considered returned, and it is not safe to make any changes on the contents of the sample.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens if a loaned data sample is not written by the user?




    :Answer:

    If function ``loan_sample()`` is called but the sample is never written, the loan must be returned to the **DataWriter** using ``discard_loan()``. Otherwise, the DataWriter may run out of samples.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Why am I unable to delete a Publisher?




    :Answer:

    To delete a **Publisher**, it is necessary to delete first all the **Entities** belonging to this **Publisher (DataWriters)**. Otherwise, the function will issue an error and the **Publisher** will not be deleted. To delete the entities, the ``delete_contained_entities`` member function must be used. This is also valid for **Subscribers**.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the meaning of the value "DATAWRITER_QOS_DEFAULT"?




    :Answer:

    The value ``DATAWRITER_QOS_DEFAULT`` has different meanings depending on where it is used. This is also applicable for the ``DATAWRITER_QOS_DEFAULT`` and ``TOPIC_QOS_DEFAULT``.
    * On ``create_datawriter()`` and ``DataWriter::set_qos()`` it refers to the default DataWriterQos as returned by ``get_default_datawriter_qos()``.
    * On ``set_default_datawriter_qos()`` it refers to the default constructed ``DataWriterQos()``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  When could a sample be removed without being received by the DataReaders?




    :Answer:

    This could happen in constrained networks or if the publication throughput is too demanding. The callback ``on_unacknowledged_sample_removed()`` can be used to detect these situations so the publishing application can apply some solution to ease this issue like reducing the publication rate.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

SUBSCRIBER
----------

.. collapse::  What is the primary consideration when accessing elements on a DDS data sequence after calling "DataReader::read()" or "DataReader::take()" operations?




    :Answer:

    After calling the ``DataReader::read()`` or ``DataReader::take()`` operations, accessing the data on the returned sequences is quite easy. The sequences API provides a ``length()`` operation returning the number of elements in the collections. The application code just needs to check this value and use the ``[]`` operator to access the corresponding elements. Elements on the DDS data sequence should only be accessed when the corresponding element on the SampleInfo sequence indicates that valid data is present. When using Data Sharing, it is also important to check that the sample is valid (i.e., not replaced, refer to DataReader and DataWriter history coupling for further information in this regard.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary method for an application to receive new data values from a DataReader without relying on a Listener?




    :Answer:

    Instead of relying on the **Listener** to try and get new data values, the application can also dedicate a thread to wait until any new data is available on the **DataReader**. This can be done using a wait-set to wait for a change on the **DataAvailable** status.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the meaning of the sequences returned by the "DataReader::read()" and "DataReader::take()" operations?




    :Answer:

    Received DDS data samples in a sequence of the data type and corresponding information about each DDS sample in a **SampleInfo** sequence.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the consequence if an application does not return loaned sequences back to the middleware?




    :Answer:

    If the application does not return the loan by calling the ``DataReader::return_loan()`` operation, then Fast DDS will eventually run out of memory to store DDS data samples received from the network for that **DataReader**.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of "disposed_generation_count" in relation to an object's lifecycle?




    :Answer:

    ``disposed_generation_count`` indicates the number of times the instance had become alive after it was disposed.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of "no_writers_generation_count" in relation to the instance's lifecycle?




    :Answer:

    ``no_writers_generation_count`` indicates the number of times the instance had become alive after it was disposed as ``NOT_ALIVE_NO_WRITERS``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of the "related_sample_identity" extension in a requester-replier configuration?




    :Answer:

    ``related_sample_identity`` is an extension for requester-replier configuration. On reply messages, it contains the sample_identity of the related request message. It is used by the requester to be able to link each reply to the appropriate request.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of setting "valid_data" to false in a data sample?




    :Answer:

    ``valid_data`` is a boolean that indicates whether the data sample contains a change in the value or not. Samples with this value set to false are used to communicate a change in the instance status, e.g., a change in the liveliness of the instance. In this case, the data sample should be dismissed as all the relevant information is in the data members of **SampleInfo**.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

TOPIC
-----

.. collapse::  What is the primary function of a Topic in the context of Publisher-Subscriber communication?




    :Answer:

    A ``Topic`` is a specialization of the broader concept of **TopicDescription**. A **Topic** represents a single data flow between **Publisher** and **Subscriber**.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What type of values can the "like" operator be used with in the context of content Filtered Topic?




    :Answer:

    The like operator is similar to the one defined by SQL. This operator can only be used with strings. There are two wildcards that could be used in conjunction with this operator.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What does the "like" operator represent in terms of character matching?




    :Answer:

    The percent sign ``%"``(or its alias ``*``) represents zero, one, or multiple characters.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What does the "like" operator's wildcard represent when used in a string comparison?




    :Answer:

    The underscore sign ``_`` (or its alias ``?``) represents one single character.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is filtering disabled?




    :Answer:

    If the expression is an empty string, that disables filtering as explained in creating a **ContentFilteredTopic**.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How to enable the usage of a Custom Filter in an application?




    :Answer:

    To be able to use the **Custom Filter** in an application, the Custom Filter's factory must be registered in the ``DomainParticipant``. The next snippet code shows how to register a factory through the API function ``register_content_filter_factory()``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What will happen if an error occurs during the creation of a ContentFilteredTopic using the "create_contentfilteredtopic()" function?




    :Answer:

    ``create_contentfilteredtopic()`` will return a null pointer if there was an error during the operation, e.g., if the related Topic belongs to a different **DomainParticipant**, a **Topic** with the same name already exists, syntax errors on the filter expression, or missing parameter values. It is advisable to check that the returned value is a valid pointer.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the method used to delete a ContentFilteredTopic in the context of a DomainParticipant?




    :Answer:

    A **ContentFilteredTopic** can be deleted with the ``delete_contentfilteredtopic()`` member function on the **DomainParticipant** instance where the **ContentFilteredTopic** was created.

|

