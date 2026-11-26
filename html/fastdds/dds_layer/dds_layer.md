<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer"></a>

<a id="id1"></a>

# DDS Layer

*eProsima Fast DDS* exposes two different APIs to interact with the communication service at different levels.
The main API is the Data Distribution Service (DDS) Data-Centric Publish-Subscribe (DCPS) Platform Independent Model
(PIM) API, or [DDS DCPS PIM](../api_reference/dds_pim/dds_pim.md#api-pim-dds-dcps-pim) for short, which is defined by the
[Data Distribution Service (DDS) version 1.4 specification](https://www.omg.org/spec/DDS/1.4), to which *Fast DDS*
complies.
This section is devoted to explain the main characteristics and modes-of-use of this API under *Fast DDS*, providing an
in depth explanation of the five modules into which it is divided:

* [Core](core/core.md#dds-layer-core):
  It defines the abstract classes and interfaces that are refined by the other modules.
  It also provides the Quality of Service (QoS) definitions, as well as support for the notification-based interaction
  style with the middleware.
* [Domain](domain/domain.md#dds-layer-domain):
  It contains the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) class that acts as an entry-point of the Service,
  as well as a factory for many of the classes. The [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) also acts as a container for the other
  objects that make up the Service.
* [Publisher](publisher/publisher.md#dds-layer-publisher):
  It describes the classes used on the publication side, including [`Publisher`](../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE) and
  [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) classes, as well as the [`PublisherListener`](../api_reference/dds_pim/publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) and
  [`DataWriterListener`](../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) interfaces.
* [Subscriber](subscriber/subscriber.md#dds-layer-subscriber):
  It describes the classes used on the subscription side, including [`Subscriber`](../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) and
  [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) classes, as well as the [`SubscriberListener`](../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE)
  and [`DataReaderListener`](../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE) interfaces.
* [Topic](topic/topic.md#dds-layer-topic):
  It describes the classes used to define communication topics and data types, including [`Topic`](../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE)
  and [`TopicDescription`](../api_reference/dds_pim/topic/topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE) classes, as well as [`TypeSupport`](../api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE), and the
  [`TopicListener`](../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) interface.

* [Core](core/core.md)
  * [Entity](core/entity/entity.md)
  * [Policy](core/policy/policy.md)
  * [Status](core/status/status.md)
  * [Conditions and Wait-sets](core/waitsets/waitsets.md)
* [Domain](domain/domain.md)
  * [DomainParticipant](domain/domainParticipant/domainParticipant.md)
  * [DomainParticipantListener](domain/domainParticipantListener/domainParticipantListener.md)
  * [CongestionControlListener <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](domain/domainParticipantListener/congestion_control_listener.md)
  * [DomainParticipantFactory](domain/domainParticipantFactory/domainParticipantFactory.md)
  * [Creating a DomainParticipant](domain/domainParticipant/createDomainParticipant.md)
  * [Partitions](domain/domainParticipant/partition.md)
* [Publisher](publisher/publisher.md)
  * [Publisher](publisher/publisher/publisher.md)
  * [PublisherListener](publisher/publisherListener/publisherListener.md)
  * [Creating a Publisher](publisher/publisher/createPublisher.md)
  * [DataWriter](publisher/dataWriter/dataWriter.md)
  * [DataWriterListener](publisher/dataWriterListener/dataWriterListener.md)
  * [Creating a DataWriter](publisher/dataWriter/createDataWriter.md)
  * [Publishing data](publisher/dataWriter/publishingData.md)
* [Subscriber](subscriber/subscriber.md)
  * [Subscriber](subscriber/subscriber/subscriber.md)
  * [SubscriberListener](subscriber/subscriberListener/subscriberListener.md)
  * [Creating a Subscriber](subscriber/subscriber/createSubscriber.md)
  * [DataReader](subscriber/dataReader/dataReader.md)
  * [DataReaderListener](subscriber/dataReaderListener/dataReaderListener.md)
  * [Creating a DataReader](subscriber/dataReader/createDataReader.md)
  * [SampleInfo](subscriber/sampleInfo/sampleInfo.md)
  * [Accessing received data](subscriber/dataReader/readingData.md)
* [Topic](topic/topic.md)
  * [Topics, keys and instances](topic/instances.md)
  * [TopicDescription](topic/topicDescription/topicDescription.md)
  * [Topic](topic/topic/topic.md)
  * [ContentFilteredTopic](topic/contentFilteredTopic/contentFilteredTopic.md)
  * [TopicListener](topic/topicListener/topicListener.md)
  * [Definition of data types](topic/typeSupport/typeSupport.md)
  * [Creating a Topic](topic/topic/createTopic.md)
  * [Filtering data on a Topic](topic/contentFilteredTopic/createContentFilteredTopic.md)
  * [The default SQL-like filter](topic/contentFilteredTopic/defaultFilter.md)
  * [Using custom filters](topic/contentFilteredTopic/customFilters.md)
  * [Where is filtering applied: writer vs reader side](topic/contentFilteredTopic/writerFiltering.md)
  * [Fast DDS-Gen for data types source code generation](topic/fastddsgen/fastddsgen.md)
