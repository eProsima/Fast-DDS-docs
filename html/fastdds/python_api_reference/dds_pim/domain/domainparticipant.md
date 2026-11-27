<a id="python-api-pim-domainparticipant"></a>

<a id="domainparticipant"></a>

# DomainParticipant

### *class* fastdds.DomainParticipant(\*args, \*\*kwargs)

Class DomainParticipant used to group Publishers and Subscribers into a single working unit.

#### assert_liveliness()

This operation manually asserts the liveliness of the DomainParticipant.
This is used in combination with the LIVELINESS QoS policy to indicate to the Service that the entity
remains active.

This operation needs to only be used if the DomainParticipant contains DataWriter entities with
the LIVELINESS set to MANUAL_BY_PARTICIPANT and it only affects the liveliness of those DataWriter entities.
Otherwise, it has no effect.

Notes: Writing data via the write operation on a DataWriter asserts liveliness on the DataWriter itself and its
DomainParticipant. Consequently the use of assert_liveliness is only needed if the application is not
writing data regularly.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the liveliness was asserted, RETCODE_ERROR otherwise.

#### contains_entity(a_handle, recursive=True)

This operation checks whether or not the given handle represents an Entity that was created from the
DomainParticipant.

* **Parameters:**
  * **a_handle** (`InstanceHandle_t`) – InstanceHandle of the entity to look for.
  * **recursive** (*boolean* *,* *optional*) – The containment applies recursively. That is, it applies both to entities
    (TopicDescription, Publisher, or Subscriber) created directly using the DomainParticipant as well as
    entities created using a contained Publisher, or Subscriber as the factory, and so forth. (default: true)
* **Return type:**
  boolean
* **Returns:**
  True if entity is contained. False otherwise.

#### create_contentfilteredtopic(\*args)

*Overload 1:*

Create a ContentFilteredTopic in this Participant.

* **Parameters:**
  * **name** (*string*) – Name of the ContentFilteredTopic
  * **related_topic** ([`Topic`](../topic/topic_class.md#fastdds.Topic)) – Related Topic to being subscribed
  * **filter_expression** (*string*) – Logic expression to create filter
  * **expression_parameters** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Parameters to filter content
  * **[****out****]** – ret_code Return code of the operation, RETCODE_OK if the ContentFilteredTopic is created, RETCODE_ERROR otherwise.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  Pointer to the created ContentFilteredTopic.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if `related_topic` does not belong to this participant.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a topic with the specified `name` has already been created.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and
  `expression_parameters`.

<br/>

*Overload 2:*

Create a ContentFilteredTopic in this Participant.

* **Parameters:**
  * **name** (*string*) – Name of the ContentFilteredTopic
  * **related_topic** ([`Topic`](../topic/topic_class.md#fastdds.Topic)) – Related Topic to being subscribed
  * **filter_expression** (*string*) – Logic expression to create filter
  * **expression_parameters** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Parameters to filter content
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  Pointer to the created ContentFilteredTopic.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if `related_topic` does not belong to this participant.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a topic with the specified `name` has already been created.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and
  `expression_parameters`.

<br/>

*Overload 3:*

Create a ContentFilteredTopic in this Participant using a custom filter.

* **Parameters:**
  * **name** (*string*) – Name of the ContentFilteredTopic
  * **related_topic** ([`Topic`](../topic/topic_class.md#fastdds.Topic)) – Related Topic to being subscribed
  * **filter_expression** (*string*) – Logic expression to create filter
  * **expression_parameters** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Parameters to filter content
  * **filter_class_name** (*string*) – Name of the filter class to use
  * **[****out****]** – ret_code Return code of the operation, RETCODE_OK if the ContentFilteredTopic is created, RETCODE_ERROR otherwise.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  Pointer to the created ContentFilteredTopic.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if `related_topic` does not belong to this participant.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a topic with the specified `name` has already been created.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and
  `expression_parameters`.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if the specified `filter_class_name` has not been registered.

<br/>

*Overload 4:*

Create a ContentFilteredTopic in this Participant using a custom filter.

* **Parameters:**
  * **name** (*string*) – Name of the ContentFilteredTopic
  * **related_topic** ([`Topic`](../topic/topic_class.md#fastdds.Topic)) – Related Topic to being subscribed
  * **filter_expression** (*string*) – Logic expression to create filter
  * **expression_parameters** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Parameters to filter content
  * **filter_class_name** (*string*) – Name of the filter class to use
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  Pointer to the created ContentFilteredTopic.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if `related_topic` does not belong to this participant.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a topic with the specified `name` has already been created.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if a filter cannot be created with the specified `filter_expression` and
  `expression_parameters`.
* **Return type:**
  `ContentFilteredTopic`
* **Returns:**
  nullptr if the specified `filter_class_name` has not been registered.

#### create_multitopic(\*args)

*Overload 1:*

Create a MultiTopic in this Participant.

* **Parameters:**
  * **name** (*string*) – Name of the MultiTopic
  * **type_name** (*string*) – Result type of the MultiTopic
  * **subscription_expression** (*string*) – Logic expression to combine filter
  * **expression_parameters** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Parameters to subscription content
  * **[****out****]** – ret_code Return code of the operation, RETCODE_OK if the MultiTopic is created, RETCODE_ERROR otherwise.
* **Return type:**
  eprosima::fastdds::dds::MultiTopic
* **Returns:**
  Pointer to the created ContentFilteredTopic, nullptr in error case

<br/>

*Overload 2:*

Create a MultiTopic in this Participant.

* **Parameters:**
  * **name** (*string*) – Name of the MultiTopic
  * **type_name** (*string*) – Result type of the MultiTopic
  * **subscription_expression** (*string*) – Logic expression to combine filter
  * **expression_parameters** (*std::vector< std::string* *,**std::allocator< std::string > >*) – Parameters to subscription content
* **Return type:**
  eprosima::fastdds::dds::MultiTopic
* **Returns:**
  Pointer to the created ContentFilteredTopic, nullptr in error case

#### create_publisher(\*args)

Create a Publisher in this Participant.

* **Parameters:**
  * **qos** ([`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)) – QoS of the Publisher.
  * **listener** ([`PublisherListener`](../publisher/publisherlistener.md#fastdds.PublisherListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Publisher`](../publisher/publisher_class.md#fastdds.Publisher)
* **Returns:**
  Pointer to the created Publisher.

#### create_publisher_with_profile(\*args)

Create a Publisher in this Participant.

* **Parameters:**
  * **profile_name** (*string*) – Publisher profile name.
  * **listener** ([`PublisherListener`](../publisher/publisherlistener.md#fastdds.PublisherListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Publisher`](../publisher/publisher_class.md#fastdds.Publisher)
* **Returns:**
  Pointer to the created Publisher.

#### create_service(\*args)

*Overload 1:*

Create an enabled RPC service.

* **Parameters:**
  * **service_name** (*string*) – Name of the service.
  * **service_type_name** (*string*) – Type name of the service (Request & reply types)
  * **[****out****]** – ret_code Return code indicating the result of the operation.
* **Return type:**
  eprosima::fastdds::dds::rpc::Service
* **Returns:**
  Pointer to the created service. nullptr in error case.

<br/>

*Overload 2:*

Create a RPC service.

* **Parameters:**
  * **service_name** (*string*) – Name of the service.
  * **service_type_name** (*string*) – Type name of the service (Request & reply types)
* **Return type:**
  eprosima::fastdds::dds::rpc::Service
* **Returns:**
  Pointer to the created service. nullptr in error case.

#### create_service_replier(\*args)

*Overload 1:*

Create a RPC Replier in a given Service.

* **Parameters:**
  * **service** (*eprosima::fastdds::dds::rpc::Service*) – Pointer to a service object where the Replier will be created.
  * **replier_qos** (`ReplierQos`) – QoS of the replier.
  * **[****out****]** – ret_code Return code indicating the result of the operation.
* **Return type:**
  eprosima::fastdds::dds::rpc::Replier
* **Returns:**
  Pointer to the created replier. nullptr in error case.

<br/>

*Overload 2:*

Create a RPC Replier in a given Service. It will override the current service’s replier

* **Parameters:**
  * **service** (*eprosima::fastdds::dds::rpc::Service*) – Pointer to a service object where the Replier will be created.
  * **replier_qos** (`ReplierQos`) – QoS of the replier.
* **Return type:**
  eprosima::fastdds::dds::rpc::Replier
* **Returns:**
  Pointer to the created replier. nullptr in error case.

#### create_service_requester(\*args)

*Overload 1:*

Create a RPC Requester in a given Service.
:type service: eprosima::fastdds::dds::rpc::Service
:param service: Pointer to a service object where the requester will be created.
:type requester_qos: `RequesterQos`
:param requester_qos: QoS of the requester.
:param [out]: ret_code Return code indicating the result of the operation.
:rtype: eprosima::fastdds::dds::rpc::Requester
:return: Pointer to the created requester. nullptr in error case.

<br/>

*Overload 2:*

Create a RPC Requester in a given Service.

* **Parameters:**
  * **service** (*eprosima::fastdds::dds::rpc::Service*) – Pointer to a service object where the requester will be created.
  * **requester_qos** (`RequesterQos`) – QoS of the requester.
* **Return type:**
  eprosima::fastdds::dds::rpc::Requester
* **Returns:**
  Pointer to the created requester. nullptr in error case.

#### create_subscriber(\*args)

Create a Subscriber in this Participant.

* **Parameters:**
  * **qos** ([`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)) – QoS of the Subscriber.
  * **listener** ([`SubscriberListener`](../subscriber/subscriberlistener.md#fastdds.SubscriberListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Subscriber`](../subscriber/subscriber_class.md#fastdds.Subscriber)
* **Returns:**
  Pointer to the created Subscriber.

#### create_subscriber_with_profile(\*args)

Create a Subscriber in this Participant.

* **Parameters:**
  * **profile_name** (*string*) – Subscriber profile name.
  * **listener** ([`SubscriberListener`](../subscriber/subscriberlistener.md#fastdds.SubscriberListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Subscriber`](../subscriber/subscriber_class.md#fastdds.Subscriber)
* **Returns:**
  Pointer to the created Subscriber.

#### create_topic(\*args)

*Overload 1:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – QoS of the Topic.
  * **[****out****]** – ret_code Return code of the operation, RETCODE_OK if the topic is created, RETCODE_ERROR otherwise.
  * **listener** ([`TopicListener`](../topic/topiclistener.md#fastdds.TopicListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

<br/>

*Overload 2:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – QoS of the Topic.
  * **listener** ([`TopicListener`](../topic/topiclistener.md#fastdds.TopicListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

<br/>

*Overload 3:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – QoS of the Topic.
  * **listener** ([`TopicListener`](../topic/topiclistener.md#fastdds.TopicListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

<br/>

*Overload 4:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – QoS of the Topic.
  * **listener** – Pointer to the listener (default: nullptr)
  * **mask** – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

#### create_topic_with_profile(\*args)

*Overload 1:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **profile_name** (*string*) – Topic profile name.
  * **[****out****]** – ret_code Return code of the operation, RETCODE_OK if the topic is created, RETCODE_ERROR otherwise.
  * **listener** ([`TopicListener`](../topic/topiclistener.md#fastdds.TopicListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

<br/>

*Overload 2:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **profile_name** (*string*) – Topic profile name.
  * **listener** ([`TopicListener`](../topic/topiclistener.md#fastdds.TopicListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

<br/>

*Overload 3:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **profile_name** (*string*) – Topic profile name.
  * **listener** ([`TopicListener`](../topic/topiclistener.md#fastdds.TopicListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

<br/>

*Overload 4:*

Create a Topic in this Participant.

* **Parameters:**
  * **topic_name** (*string*) – Name of the Topic.
  * **type_name** (*string*) – Data type of the Topic.
  * **profile_name** (*string*) – Topic profile name.
  * **listener** – Pointer to the listener (default: nullptr)
  * **mask** – StatusMask that holds statuses the listener responds to (default: all)
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the created Topic.

#### delete_contained_entities()

Deletes all the entities that were created by means of the “create” methods

* **Return type:**
  int
* **Returns:**
  RETURN_OK code if everything correct, error code otherwise

#### delete_contentfilteredtopic(a_contentfilteredtopic)

Deletes an existing ContentFilteredTopic.

* **Parameters:**
  **a_contentfilteredtopic** (`ContentFilteredTopic`) – ContentFilteredTopic to be deleted
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic passed is a nullptr, RETCODE_PRECONDITION_NOT_MET if the topic does not belong to
  this participant or if it is referenced by any entity and RETCODE_OK if the ContentFilteredTopic was deleted.

#### delete_multitopic(a_multitopic)

Deletes an existing MultiTopic.

* **Parameters:**
  **a_multitopic** (*eprosima::fastdds::dds::MultiTopic*) – MultiTopic to be deleted
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic passed is a nullptr, RETCODE_PRECONDITION_NOT_MET if the topic does not belong to
  this participant or if it is referenced by any entity and RETCODE_OK if the Topic was deleted.

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### delete_publisher(publisher)

Deletes an existing Publisher.

* **Parameters:**
  **publisher** ([`Publisher`](../publisher/publisher_class.md#fastdds.Publisher)) – to be deleted.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the publisher does not belong to this participant or if it has active DataWriters,
  RETCODE_OK if it is correctly deleted and RETCODE_ERROR otherwise.

#### delete_service(service)

Delete a registered RPC service

* **Parameters:**
  **service** (*eprosima::fastdds::dds::rpc::Service*) – Pointer to the service to be deleted.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the service was deleted, or an specific error code otherwise.

#### delete_service_replier(service_name, replier)

Deletes an existing RPC Replier

* **Parameters:**
  * **service_name** (*string*) – Name of the service where the replier is created.
  * **replier** (*eprosima::fastdds::dds::rpc::Replier*) – Pointer to the replier to be deleted.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the replier was deleted, or an specific error code otherwise.

#### delete_service_requester(service_name, requester)

Deletes an existing RPC Requester

* **Parameters:**
  * **service_name** (*string*) – Name of the service where the requester is created.
  * **requester** (*eprosima::fastdds::dds::rpc::Requester*) – Pointer to the requester to be deleted.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the requester was deleted, or an specific error code otherwise.

#### delete_subscriber(subscriber)

Deletes an existing Subscriber.

* **Parameters:**
  **subscriber** ([`Subscriber`](../subscriber/subscriber_class.md#fastdds.Subscriber)) – to be deleted.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the subscriber does not belong to this participant or if it has active DataReaders,
  RETCODE_OK if it is correctly deleted and RETCODE_ERROR otherwise.

#### delete_topic(topic)

Deletes an existing Topic.

* **Parameters:**
  **topic** ([`Topic`](../topic/topic_class.md#fastdds.Topic)) – to be deleted.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic passed is a nullptr, RETCODE_PRECONDITION_NOT_MET if the topic does not belong to
  this participant or if it is referenced by any entity and RETCODE_OK if the Topic was deleted.

#### disable_monitor_service()

\_pro Disables the monitor service in this DomainParticipant.
Does nothing if the service was not enabled before.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the monitor service could be correctly disabled.
* **Return type:**
  int
* **Returns:**
  RETCODE_NOT_ENABLED if the monitor service was not previously enabled.
* **Return type:**
  int
* **Returns:**
  RETCODE_ERROR if the service could not be properly disabled.

#### disable_statistics_datawriter(topic_name)

\_pro This operation disables a Statistics DataWriter

* **Parameters:**
  **topic_name** (*string*) – Name of the topic associated to the Statistics DataWriter
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic name provided does not correspond to any Statistics DataWriter,
  RETCODE_OK if the DataWriter has been correctly deleted or does not exist,
  and RETCODE_ERROR otherwise

#### enable()

This operation enables the DomainParticipant

* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### enable_monitor_service()

\_pro Enables the monitor service in the DomainParticipant.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the monitor service could be correctly enabled.
* **Return type:**
  int
* **Returns:**
  RETCODE_ERROR if the monitor service could not be enabled properly.

#### enable_statistics_datawriter(topic_name, dwqos)

\_pro This operation enables a Statistics DataWriter

* **Parameters:**
  * **[****in****]** – topic_name Name of the topic associated to the Statistics DataWriter
  * **[****in****]** – dwqos DataWriterQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic name provided does not correspond to any Statistics DataWriter,
  RETCODE_INCONSISTENT_POLICY if the DataWriterQos provided are inconsistent,
  RETCODE_OK if the DataWriter has been created or if it has been created previously,
  and RETCODE_ERROR otherwise

#### enable_statistics_datawriter_with_profile(profile_name, topic_name)

\_pro This operation enables a Statistics DataWriter from a given profile

* **Parameters:**
  * **[****in****]** – profile_name DataWriter QoS profile name
  * **[****in****]** – topic_name Name of the statistics topic to be enabled.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the topic name provided does not correspond to any Statistics DataWriter,
  RETCODE_INCONSISTENT_POLICY if the DataWriterQos provided in profile are inconsistent,
  RETCODE_OK if the DataWriter has been created or if it has been created previously,
  and RETCODE_ERROR otherwise

#### fill_discovery_data_from_cdr_message(\*args)

*Overload 1:*

\_pro fills in the ParticipantBuiltinTopicData from a MonitorService Message

* **Parameters:**
  * **[****out****]** – data Proxy to fill
  * **[****in****]** – msg MonitorService Message to get the proxy information from.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the operation succeeds.
* **Return type:**
  int
* **Returns:**
  RETCODE_ERROR if the  operation fails.

<br/>

*Overload 2:*

\_pro fills in the PublicationBuiltinTopicData from a MonitorService Message

* **Parameters:**
  * **[****out****]** – data Proxy to fill.
  * **[****in****]** – msg MonitorService Message to get the proxy information from.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the operation succeeds.
* **Return type:**
  int
* **Returns:**
  RETCODE_ERROR if the  operation fails.

<br/>

*Overload 3:*

\_pro fills in the SubscriptionBuiltinTopicData from a MonitorService Message

* **Parameters:**
  * **[****out****]** – data Proxy to fill.
  * **[****in****]** – msg MonitorService Message to get the proxy information from.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the operation succeeds.
* **Return type:**
  int
* **Returns:**
  RETCODE_ERROR if the  operation fails.

#### find_service(service_name)

Find a RPC service by name

* **Parameters:**
  **service_name** (*string*) – Name of the service to search for.
* **Return type:**
  eprosima::fastdds::dds::rpc::Service
* **Returns:**
  Pointer to the service object if found, nullptr if not found.

#### find_service_type(service_type_name)

This method gives access to a registered service type based on its name.

* **Parameters:**
  **service_type_name** (*string*) – Name of the type
* **Return type:**
  eprosima::fastdds::dds::rpc::ServiceTypeSupport
* **Returns:**
  ServiceTypeSupport corresponding to the service_type_name

#### find_topic(topic_name, timeout)

Gives access to an existing (or ready to exist) enabled Topic.
It should be noted that the returned Topic is a local object that acts as a proxy to designate the global
concept of topic.
Topics obtained by means of find_topic, must also be deleted by means of delete_topic so that the local
resources can be released.
If a Topic is obtained multiple times by means of find_topic or create_topic, it must also be deleted that same
number of times using delete_topic.

* **Parameters:**
  * **topic_name** (*string*) – Topic name
  * **timeout** (`Duration_t`) – Maximum time to wait for the Topic
* **Return type:**
  [`Topic`](../topic/topic_class.md#fastdds.Topic)
* **Returns:**
  Pointer to the existing Topic, nullptr in case of error or timeout

#### find_type(type_name)

This method gives access to a registered type based on its name.

* **Parameters:**
  **type_name** (*string*) – Name of the type
* **Return type:**
  [`TypeSupport`](../topic/typesupport.md#fastdds.TypeSupport)
* **Returns:**
  TypeSupport corresponding to the type_name

#### get_builtin_subscriber()

Allows access to the builtin Subscriber.

* **Return type:**
  [`Subscriber`](../subscriber/subscriber_class.md#fastdds.Subscriber)
* **Returns:**
  Pointer to the builtin Subscriber, nullptr in error case

#### get_current_time(current_time)

This operation returns the current value of the time that the service uses to time-stamp data-writes
and to set the reception-timestamp for the data-updates it receives.

* **Parameters:**
  **current_time** (`Time_t`) – Time_t reference where the current time is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_default_publisher_qos(\*args)

*Overload 1:*

This operation retrieves the default value of the Publisher QoS, that is, the QoS policies which will be used
for newly created Publisher entities in the case where the QoS policies are defaulted in the
create_publisher operation.

The values retrieved get_default_publisher_qos will match the set of values specified on the last successful
call to set_default_publisher_qos, or else, if the call was never made, the default values.

* **Return type:**
  [`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)
* **Returns:**
  Current default publisher qos.

<br/>

*Overload 2:*

This operation retrieves the default value of the Publisher QoS, that is, the QoS policies which will be used
for newly created Publisher entities in the case where the QoS policies are defaulted in the
create_publisher operation.

The values retrieved get_default_publisher_qos will match the set of values specified on the last successful
call to set_default_publisher_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** ([`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)) – PublisherQos reference where the default_publisher_qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_default_publisher_qos_from_xml(xml, qos)

Fills the ‘PublisherQos’ with the default publisher profile found in the provided XML (if there is).

Notes: This method does not update the default publisher qos (returned by `get_default_publisher_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)) – ‘PublisherQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_default_replier_qos_from_xml(xml, qos)

Fills the ‘ReplierQos’ with the default replier profile found in the provided XML (if there is).

Notes: This method does not update the default replier qos.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** (`ReplierQos`) – ‘ReplierQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_default_requester_qos_from_xml(xml, qos)

Fills the ‘RequesterQos’ with the default requester profile found in the provided XML (if there is).

Notes: This method does not update the default requester qos.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** (`RequesterQos`) – ‘RequesterQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_default_subscriber_qos(\*args)

*Overload 1:*

This operation retrieves the default value of the Subscriber QoS, that is, the QoS policies which will be used
for newly created Subscriber entities in the case where the QoS policies are defaulted in the
create_subscriber operation.

The values retrieved get_default_subscriber_qos will match the set of values specified on the last successful
call to set_default_subscriber_qos, or else, if the call was never made, the default values.

* **Return type:**
  [`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)
* **Returns:**
  Current default subscriber qos.

<br/>

*Overload 2:*

This operation retrieves the default value of the Subscriber QoS, that is, the QoS policies which will be used
for newly created Subscriber entities in the case where the QoS policies are defaulted in the
create_subscriber operation.

The values retrieved get_default_subscriber_qos will match the set of values specified on the last successful
call to set_default_subscriber_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** ([`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)) – SubscriberQos reference where the default_subscriber_qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_default_subscriber_qos_from_xml(xml, qos)

Fills the ‘SubscriberQos’ with the default subscriber profile found in the provided XML (if there is).

Notes: This method does not update the default subscriber qos (returned by `get_default_subscriber_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)) – ‘SubscriberQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_default_topic_qos(\*args)

*Overload 1:*

This operation retrieves the default value of the Topic QoS, that is, the QoS policies that will be used
for newly created Topic entities in the case where the QoS policies are defaulted in the create_topic
operation.

The values retrieved get_default_topic_qos will match the set of values specified on the last successful
call to set_default_topic_qos, or else, TOPIC_QOS_DEFAULT if the call was never made.

* **Return type:**
  [`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)
* **Returns:**
  Current default topic qos.

<br/>

*Overload 2:*

This operation retrieves the default value of the Topic QoS, that is, the QoS policies that will be used
for newly created Topic entities in the case where the QoS policies are defaulted in the create_topic
operation.

The values retrieved get_default_topic_qos will match the set of values specified on the last successful
call to set_default_topic_qos, or else, TOPIC_QOS_DEFAULT if the call was never made.

* **Parameters:**
  **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – TopicQos reference where the default_topic_qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_default_topic_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘TopicQos’ with the default topic profile found in the provided XML (if there is).

Notes: This method does not update the default topic qos (returned by `get_default_topic_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘TopicQos’ with the default topic profile found in the provided XML (if there is), and also its corresponding topic and data type names (if specified).

Notes: This method does not update the default topic qos (returned by `get_default_topic_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
  * **topic_data_type** (*string*) – String where the name of the topic data type associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_discovered_participant_data(participant_data, participant_handle)

Retrieves the DomainParticipant data of a discovered not ignored participant.

* **Parameters:**
  * **[****out****]** – participant_data Reference to the ParticipantBuiltinTopicData object to return the data
  * **participant_handle** (`InstanceHandle_t`) – InstanceHandle of DomainParticipant to retrieve the data from
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if everything correct, PRECONDITION_NOT_MET if participant does not exist

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_discovered_participants(participant_handles)

Retrieves the list of DomainParticipants that have been discovered in the domain and are not “ignored”.

* **Parameters:**
  **[****out****]** – participant_handles Reference to the vector where discovered participants will be returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if everything correct, error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_discovered_topic_data(topic_data, topic_handle)

Retrieves the Topic data of a discovered not ignored topic.

* **Parameters:**
  * **[****out****]** – topic_data Reference to the TopicBuiltinTopicData object to return the data
  * **topic_handle** (`InstanceHandle_t`) – InstanceHandle of Topic to retrieve the data from
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if everything correct, PRECONDITION_NOT_MET if topic does not exist

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_discovered_topics(topic_handles)

Retrieves the list of topics that have been discovered in the domain and are not “ignored”.

* **Parameters:**
  **[****out****]** – topic_handles Reference to the vector where discovered topics will be returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if everything correct, error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_domain_id()

This operation retrieves the domain_id used to create the DomainParticipant.
The domain_id identifies the DDS domain to which the DomainParticipant belongs.

* **Return type:**
  int
* **Returns:**
  The Participant’s domain_id

#### get_instance_handle()

Returns the DomainParticipant’s handle.

* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  InstanceHandle of this DomainParticipant.

#### get_listener()

Allows accessing the DomainParticipantListener.

* **Return type:**
  [`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener)
* **Returns:**
  DomainParticipantListener pointer

#### get_participant_names()

Getter for the participant names

* **Return type:**
  std::vector< std::string,std::allocator< std::string > >
* **Returns:**
  Vector with the names

#### get_publisher_qos_from_profile(profile_name, qos)

Fills the ‘PublisherQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – Publisher profile name.
  * **qos** ([`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)) – ‘PublisherQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_publisher_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘PublisherQos’ with the first publisher profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)) – ‘PublisherQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘PublisherQos’ with the publisher profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)) – ‘PublisherQos’ object where the qos is returned.
  * **profile_name** (*string*) – Publisher profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_qos(\*args)

*Overload 1:*

This operation returns the value of the DomainParticipant QoS policies

* **Parameters:**
  **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – DomainParticipantQos reference where the qos is going to be returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

<br/>

*Overload 2:*

This operation returns the value of the DomainParticipant QoS policies

* **Return type:**
  [`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)
* **Returns:**
  A reference to the DomainParticipantQos

#### get_replier_qos_from_profile(profile_name, qos)

Fills the ‘ReplierQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – Replier profile name.
  * **qos** (`ReplierQos`) – ‘ReplierQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_replier_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘ReplierQos’ with the first replier profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** (`ReplierQos`) – ‘ReplierQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘ReplierQos’ with the replier profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** (`ReplierQos`) – ‘ReplierQos’ object where the qos is returned.
  * **profile_name** (*string*) – Replier profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_requester_qos_from_profile(profile_name, qos)

Fills the ‘RequesterQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – Requester profile name.
  * **qos** (`RequesterQos`) – ‘RequesterQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_requester_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘RequesterQos’ with the first requester profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** (`RequesterQos`) – ‘RequesterQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘RequesterQos’ with the requester profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** (`RequesterQos`) – ‘RequesterQos’ object where the qos is returned.
  * **profile_name** (*string*) – Requester profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_subscriber_qos_from_profile(profile_name, qos)

Fills the ‘SubscriberQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – Subscriber profile name.
  * **qos** ([`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)) – ‘SubscriberQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_subscriber_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘SubscriberQos’ with the first subscriber profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)) – ‘SubscriberQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘SubscriberQos’ with the subscriber profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)) – ‘SubscriberQos’ object where the qos is returned.
  * **profile_name** (*string*) – Subscriber profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_topic_qos_from_profile(\*args)

*Overload 1:*

Fills the ‘TopicQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – Topic profile name.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘TopicQos’ with the values of the XML profile, and also its corresponding topic and data type names (if specified).

* **Parameters:**
  * **profile_name** (*string*) – Topic profile name.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
  * **topic_data_type** (*string*) – String where the name of the topic data type associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_topic_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘TopicQos’ with the first topic profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘TopicQos’ with the first topic profile found in the provided XML, and also its corresponding topic and data type names (if specified).

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
  * **topic_data_type** (*string*) – String where the name of the topic data type associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 3:*

Fills the ‘TopicQos’ with the topic profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
  * **profile_name** (*string*) – Topic profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 4:*

Fills the ‘TopicQos’ with the topic profile with `profile_name` to be found in the provided XML, and also its corresponding topic and data type names (if specified).

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – ‘TopicQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
  * **topic_data_type** (*string*) – String where the name of the topic data type associated to this profile is returned (if specified).
  * **profile_name** (*string*) – Topic profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### guid()

Getter for the Participant GUID

* **Return type:**
  `GUID_t`
* **Returns:**
  A reference to the GUID

#### ignore_participant(handle)

Locally ignore a remote domain participant.

Notes: This action is not reversible.

* **Parameters:**
  **handle** (`InstanceHandle_t`) – Identifier of the remote participant to ignore
* **Return type:**
  int
* **Returns:**
  RETURN_OK code if everything correct, RETCODE_BAD_PARAMENTER otherwise

#### ignore_publication(handle)

Locally ignore a remote datawriter.

Notes: This action is not reversible.

* **Parameters:**
  **handle** (`InstanceHandle_t`) – Identifier of the datawriter to ignore
* **Return type:**
  int
* **Returns:**
  RETURN_OK code if everything correct, error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### ignore_subscription(handle)

Locally ignore a remote datareader.

Notes: This action is not reversible.

* **Parameters:**
  **handle** (`InstanceHandle_t`) – Identifier of the datareader to ignore
* **Return type:**
  int
* **Returns:**
  RETURN_OK code if everything correct, error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### ignore_topic(handle)

Locally ignore a topic.

Notes: This action is not reversible.

* **Parameters:**
  **handle** (`InstanceHandle_t`) – Identifier of the topic to ignore
* **Return type:**
  int
* **Returns:**
  RETURN_OK code if everything correct, error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### lookup_content_filter_factory(filter_class_name)

Lookup a custom content filter factory previously registered with register_content_filter_factory.

* **Parameters:**
  **filter_class_name** (*string*) – Name of the filter class. Cannot be nullptr.
* **Return type:**
  `IContentFilterFactory`
* **Returns:**
  nullptr if the given filter_class_name has not been previously registered on this DomainParticipant.
  Otherwise, the content filter factory previously registered with the given filter_class_name.

#### lookup_topicdescription(topic_name)

Looks up an existing, locally created ‘TopicDescription’, based on its name.
May be called on a disabled participant.

* **Parameters:**
  **topic_name** (*string*) – Name of the ‘TopicDescription’ to search for.
* **Return type:**
  [`TopicDescription`](../topic/topicdescription.md#fastdds.TopicDescription)
* **Returns:**
  Pointer to the topic description, if it has been created locally. Otherwise, nullptr is returned.

Remarks: UNSAFE. It is unsafe to lookup a topic description while another thread is creating a topic.

#### *static* narrow(\*args)

*Overload 1:*

\_pro Kept for backwards compatibility, as the statistics API is now part of this class.

* **Parameters:**
  **domain_participant** ([`DomainParticipant`](#fastdds.DomainParticipant)) – Reference to the DomainParticipant
* **Return type:**
  [`DomainParticipant`](#fastdds.DomainParticipant)
* **Returns:**
  The same DomainParticipant that was provided.

<br/>

*Overload 2:*

\_pro Kept for backwards compatibility, as the statistics API is now part of this class.

* **Parameters:**
  **domain_participant** ([`DomainParticipant`](#fastdds.DomainParticipant)) – Constant reference to the DomainParticipant
* **Return type:**
  [`DomainParticipant`](#fastdds.DomainParticipant)
* **Returns:**
  The same DomainParticipant that was provided.

#### new_remote_endpoint_discovered(partguid, userId, kind)

This method can be used when using a StaticEndpointDiscovery mechanism different that the one
included in Fast DDS, for example when communicating with other implementations.
It indicates the Participant that an Endpoint from the XML has been discovered and
should be activated.

* **Parameters:**
  * **partguid** (`GUID_t`) – Participant GUID_t.
  * **userId** (*int*) – User defined ID as shown in the XML file.
  * **kind** (*int*) – EndpointKind (WRITER or READER)
* **Return type:**
  boolean
* **Returns:**
  True if correctly found and activated.

#### register_content_filter_factory(filter_class_name, filter_factory)

Register a custom content filter factory, which can be used to create a ContentFilteredTopic.

DDS specifies a SQL-like content filter to be used by content filtered topics.
If this filter does not meet your filtering requirements, you can register a custom filter factory.

To use a custom filter, a factory for it must be registered in the following places:

- In any application that uses the custom filter factory to create a ContentFilteredTopic and the corresponding
  DataReader.
- In each application that writes the data to the applications mentioned above.

For example, suppose Application A on the subscription side creates a Topic named X and a ContentFilteredTopic
named filteredX (and a corresponding DataReader), using a previously registered content filter factory, myFilterFactory.
With only that, you will have filtering at the subscription side.
If you also want to perform filtering in any application that publishes Topic X, then you also need to register
the same definition of the ContentFilterFactory myFilterFactory in that application.

Each `filter_class_name` can only be used to register a content filter factory once per DomainParticipant.

* **Parameters:**
  * **filter_class_name** (*string*) – Name of the filter class. Cannot be nullptr, must not exceed 255 characters, and must
    be unique within this DomainParticipant.
  * **filter_factory** (`IContentFilterFactory`) – Factory of content filters to be registered. Cannot be nullptr.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if any parameter is nullptr, or the filter_class_name exceeds 255 characters.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the filter_class_name has been already registered.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if filter_class_name is FASTDDS_SQLFILTER_NAME.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the filter is correctly registered.

#### register_service_type(service_type, service_type_name)

Register a service type in this participant.

* **Parameters:**
  * **service_type** (*eprosima::fastdds::dds::rpc::ServiceTypeSupport*) – ServiceTypeSupport.
  * **service_type_name** (*string*) – The name that will be used to identify the service type.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if it is correctly registered. Error code otherwise.

#### register_type(\*args)

*Overload 1:*

Register a type in this participant.

* **Parameters:**
  * **type** ([`TypeSupport`](../topic/typesupport.md#fastdds.TypeSupport)) – TypeSupport.
  * **type_name** (*string*) – The name that will be used to identify the Type.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the size of the name is 0, RERCODE_PRECONDITION_NOT_MET if there is another TypeSupport
  with the same name and RETCODE_OK if it is correctly registered.

<br/>

*Overload 2:*

Register a type in this participant.

* **Parameters:**
  **type** ([`TypeSupport`](../topic/typesupport.md#fastdds.TypeSupport)) – TypeSupport.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the size of the name is 0, RERCODE_PRECONDITION_NOT_MET if there is another TypeSupport
  with the same name and RETCODE_OK if it is correctly registered.

#### set_congestion_control_listener(listener)

\_pro Sets the listener that observes congestion-control meta-information.

Congestion control is participant-scoped (one controller per participant covering
all reliable writers, evaluated per remote reader). The listener is purely
observational and does not change congestion-control behavior; it only takes effect
when congestion control is enabled through the participant’s properties.

* **Parameters:**
  **listener** (*eprosima::fastdds::rtps::CongestionControlListener*) – New congestion-control listener, or nullptr to detach.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful.

Warning: The listener object must outlive the participant; it is not copied.

#### set_default_publisher_qos(qos)

This operation sets a default value of the Publisher QoS policies which will be used for newly created
Publisher entities in the case where the QoS policies are defaulted in the create_publisher operation.

This operation will check that the resulting policies are self consistent; if they are not,
the operation will have no effect and return false.

The special value PUBLISHER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS
should be reset back to the initial values the factory would use, that is the values that would be used
if the set_default_publisher_qos operation had never been called.

* **Parameters:**
  **qos** ([`PublisherQos`](../publisher/publisherqos.md#fastdds.PublisherQos)) – PublisherQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly.

#### set_default_subscriber_qos(qos)

This operation sets a default value of the Subscriber QoS policies that will be used for newly created
Subscriber entities in the case where the QoS policies are defaulted in the create_subscriber operation.

This operation will check that the resulting policies are self consistent; if they are not,
the operation will have no effect and return false.

The special value SUBSCRIBER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS
should be reset back to the initial values the factory would use, that is the values that would be used
if the set_default_subscriber_qos operation had never been called.

* **Parameters:**
  **qos** ([`SubscriberQos`](../subscriber/subscriberqos.md#fastdds.SubscriberQos)) – SubscriberQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly.

#### set_default_topic_qos(qos)

This operation sets a default value of the Topic QoS policies which will be used for newly created
Topic entities in the case where the QoS policies are defaulted in the create_topic operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation
will have no effect and return INCONSISTENT_POLICY.

The special value TOPIC_QOS_DEFAULT may be passed to this operation to indicate that the default QoS
should be reset back to the initial values the factory would use, that is the values that would be used
if the set_default_topic_qos operation had never been called.

* **Parameters:**
  **qos** ([`TopicQos`](../topic/topicqos.md#fastdds.TopicQos)) – TopicQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly.

#### set_listener(\*args)

*Overload 1:*

Modifies the DomainParticipantListener, sets the mask to StatusMask::all()

* **Parameters:**
  **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener)) – new value for the DomainParticipantListener
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

<br/>

*Overload 2:*

Modifies the DomainParticipantListener.

* **Parameters:**
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener)) – new value for the DomainParticipantListener
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask)) – StatusMask that holds statuses the listener responds to
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### set_qos(qos)

This operation sets the value of the DomainParticipant QoS policies.

* **Parameters:**
  **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – DomainParticipantQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not
  self consistent and RETCODE_OK if the qos is changed correctly.

#### *property* thisown

The membership flag

#### unregister_content_filter_factory(filter_class_name)

Unregister a custom content filter factory previously registered with register_content_filter_factory.

A filter_class_name can be unregistered only if it has been previously registered to the DomainParticipant with
register_content_filter_factory.

The unregistration of filter is not allowed if there are any existing ContentFilteredTopic objects that are
using the filter.

If there is any existing discovered DataReader with the same filter_class_name, filtering on the writer side will be
stopped, but this operation will not fail.

* **Parameters:**
  **filter_class_name** (*string*) – Name of the filter class. Cannot be nullptr.
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the filter_class_name is nullptr.
* **Return type:**
  int
* **Returns:**
  RERCODE_PRECONDITION_NOT_MET if the filter_class_name has not been previously registered.
* **Return type:**
  int
* **Returns:**
  RERCODE_PRECONDITION_NOT_MET if there is any ContentFilteredTopic referencing the filter.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the filter is correctly unregistered.

#### unregister_service_type(service_type_name)

Unregister a service type in this participant.

* **Parameters:**
  **service_type_name** (*string*) – Name of the type
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if it is correctly unregistered. Error code otherwise.

#### unregister_type(typeName)

Unregister a type in this participant.

* **Parameters:**
  **typeName** (*string*) – Name of the type
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the size of the name is 0, RERCODE_PRECONDITION_NOT_MET if there are entities using that
  TypeSupport and RETCODE_OK if it is correctly unregistered.
