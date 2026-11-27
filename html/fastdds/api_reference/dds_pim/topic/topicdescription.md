<a id="api-pim-topicdescription"></a>

<a id="topicdescription"></a>

# TopicDescription

### class TopicDescription

Class [TopicDescription](#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription), represents the fact that both publications and subscriptions are tied to a single data-type 

Subclassed by [eprosima::fastdds::dds::ContentFilteredTopic](contentfilteredtopic.md#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic), [eprosima::fastdds::dds::Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic)

### Public Functions

### virtual [DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*get_participant() const = 0

Get the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) to which the [TopicDescription](#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription) belongs.

* **Returns:**
  The [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) to which the [TopicDescription](#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription) belongs. 

### inline const std::string &get_name() const

Get the name used to create this [TopicDescription](#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription).

* **Returns:**
  the name used to create this [TopicDescription](#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription). 

### inline const std::string &get_type_name() const

Get the associated type name.

* **Returns:**
  the type name.
