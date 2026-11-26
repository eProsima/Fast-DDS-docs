<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-contentfilteredtopic"></a>

<a id="contentfilteredtopic"></a>

# ContentFilteredTopic

A [`ContentFilteredTopic`](../../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) is a specialization of the broader concept of [TopicDescription](../topicDescription/topicDescription.md#dds-layer-topic-topicdescription).
A ContentFilteredTopic is a Topic with filtering properties.
It makes it possible to subscribe to a Topic while at the same time specify interest on a subset of the Topic’s data.

#### IMPORTANT
Note that a ContentFilteredTopic can only be used to create a DataReader, not a DataWriter.

A ContentFilteredTopic provides a relationship between a [Topic](../topic/topic.md#dds-layer-topic-topic), called the related topic, and
some user-defined filtering properties:

* A **filter expression**, which establishes a logical expression on the content of the related topic.
  It is similar to the WHERE clause in a SQL statement.
* A list of **expression parameters**, which give values to the parameters present in the filter expression.
  There must be one parameter string for each parameter in the filter expression.

Note that a ContentFilteredTopic is *not* an Entity, and thus it has neither QoS nor listener.
A DataReader created with a ContentFilteredTopic will use the QoS from the related topic.
Multiple DataReaders can be created for the same ContentFilteredTopic, and changing the filter properties of a
ContentFilteredTopic will affect all DataReaders using it.

Please refer to [Filtering data on a Topic](createContentFilteredTopic.md#dds-layer-topic-filtering-data-on-topic) and
[Where is filtering applied: writer vs reader side](writerFiltering.md#dds-layer-topic-contentfilteredtopic-writer-side) for more information about how to use
[`ContentFilteredTopic`](../../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE).
