<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="topic"></a>

<a id="topic-profiles"></a>

# Topic profiles

The topic profiles allow for configuring [Topic](../dds_layer/topic/topic.md#dds-layer-topic) from an XML file.
These profiles are defined within the `<topic>` XML tags.

<a id="topic-xml-attributes"></a>

## Topic XML attributes

The `<topic>` element has two attributes defined: `profile_name` and `is_default_profile`.

| Name                 | Description                                                                                                                                                                                                                                                                                                                   | Use       |
|----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| `profile_name`       | Sets the name under which the `<topic>` profile is registered in the DDS Domain,<br/>so that it can be loaded later by the [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) or the [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) | Mandatory |
| `is_default_profile` | Sets the `<topic>` profile as the default profile. Thus, if a default profile<br/>exists, it will be used when no other Topic profile is specified at the<br/>Topic’s creation.                                                                                                                                               | Optional  |

<a id="topic-configuration"></a>

## Topic configuration

This XML element allows the configuration of the [TopicQos](../dds_layer/topic/topic/topic.md#dds-layer-topic-topicqos).

| Name                  | Description                                                                                                                                                     | Values                                |
|-----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------|
| `<historyQos>`        | It controls the behavior of *Fast DDS*<br/>when the value of an instance changes<br/>before it is finally communicated to<br/>some of its existing DataReaders. | [HistoryQoS](common.md#hqos)          |
| `<resourceLimitsQos>` | It controls the resources that *Fast DDS*<br/>can use in order to meet the<br/>requirements imposed by the application<br/>and other QoS settings.              | [ResourceLimitsQos](common.md#rlsqos) |

**Example**

```xml
<topic profile_name="topic_example">
    <historyQos>
        <kind>KEEP_LAST</kind>
        <depth>20</depth>
    </historyQos>
    <resourceLimitsQos>
        <max_samples>5</max_samples>
        <max_instances>2</max_instances>
        <max_samples_per_instance>1</max_samples_per_instance>
        <allocated_samples>20</allocated_samples>
        <extra_samples>10</extra_samples>
    </resourceLimitsQos>
</topic>
```
