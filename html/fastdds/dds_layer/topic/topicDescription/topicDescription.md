<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-topicdescription"></a>

<a id="topicdescription"></a>

# TopicDescription

[`TopicDescription`](../../../api_reference/dds_pim/topic/topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE) is an abstract class that serves as the base for all classes describing a data flow.
Applications will not create instances of [`TopicDescription`](../../../api_reference/dds_pim/topic/topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE) directly, they must create instances of one
of its specializations instead.
At the moment, the only specializations implemented are [Topic](../topic/topic.md#dds-layer-topic-topic),
and [ContentFilteredTopic](../contentFilteredTopic/contentFilteredTopic.md#dds-layer-topic-contentfilteredtopic).
