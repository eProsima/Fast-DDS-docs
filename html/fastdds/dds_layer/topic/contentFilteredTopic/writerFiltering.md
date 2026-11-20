<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-contentfilteredtopic-writer-side"></a>

<a id="where-is-filtering-applied-writer-vs-reader-side"></a>

# Where is filtering applied: writer vs reader side

[Content filters](contentFilteredTopic.md#dds-layer-topic-contentfilteredtopic) may be evaluated on either side, as the DataWriter obtains
the filter expression from the DataReader during discovery.
Filtering on the writer side can save network bandwidth at the cost of increasing CPU usage on the writer.

<a id="dds-layer-topic-contentfilteredtopic-writer-restrictions"></a>

<a id="conditions-for-writer-side-filtering"></a>

## Conditions for writer side filtering

A DataWriter will perform filter evaluation in the DataReader stead whenever all of the following conditions are met;
filtering will otherwise be performed by the DataReader.

- The DataWriter has infinite liveliness.
  See [LivelinessQosPolicy](../../core/policy/standardQosPolicies.md#livelinessqospolicy).
- Communication with the DataReader is neither [intra-process](../../../transport/intraprocess.md#intraprocess-delivery) nor
  [data-sharing](../../../transport/datasharing.md#datasharing-delivery).
- The DataReader is not using multicast.
- The DataWriter is filtering for no more DataReaders than the maximum value set on
  [`reader_filters_allocation`](../../../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQos25reader_filters_allocationE).
  - There is a [resource-limit policy](../../core/policy/eprosimaExtensions.md#writerresourcelimitsqos) on [DataWriterQos](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos) that controls the allocation
    behavior of writer-side filtering resources.
    Setting a maximum value of 0 disables filter evaluation on the writer side.
    A maximum value of 32 (the default value) means the writer will perform filter evaluation for up to 32 readers.
  - If the DataWriter is evaluating filters for `writer_resource_limits.reader_filters_allocation.maximum`
    DataReaders, and a new filtered DataReader is created, then the filter for the newly created DataReader
    will be evaluated on the reader side.

<a id="dds-layer-topic-contentfilteredtopic-writer-race-condition"></a>

<a id="discovery-race-condition"></a>

## Discovery race condition

On applications where the filter expression and/or the expression parameters are updated, there may be a
situation where the DataWriter will apply the old version of the filter until it receives updated information
through discovery.
This may imply that a publication made a short time after the DataReader updated the filter, but before the
updated discovery information is received by the DataWriter, may not be sent to the DataReader, even if the
new filter would have told otherwise.
Publications made after the updated discovery information is received will use the updated filter.

If some critical application considers this race condition issue unbearable, filtering on the writer side
can be disabled by setting the maximum value on [`reader_filters_allocation`](../../../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQos25reader_filters_allocationE) to 0.

<a id="dds-layer-topic-contentfilteredtopic-writer-side-late-joiners"></a>

<a id="writer-side-filtering-on-late-joiners-pro"></a>

## Writer side filtering on late joiners **[Pro]**

By default, when a late joiner DataReader is discovered (non-volatile durability scenario), the DataWriter will not
apply the filter for that DataReader on historical samples.
This means that the late joiner DataReader will receive all historical samples, even those that would not match its
filter, which could cause performance degradation on resource-limited systems.

It is possible however to configure a DataWriter to perform writer-side filtering on late joiners by setting the
[DataWriterQos](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos) property `fastdds.content_filtering_on_late_joiners` to `true`.

| PropertyPolicyQos name                        | PropertyPolicyQos value   | Default value   |
|-----------------------------------------------|---------------------------|-----------------|
| `"fastdds.content_filtering_on_late_joiners"` | `bool`                    | `false`         |

### C++

```c++
DataWriterQos wqos;

// Enable content filtering on late joiners for this DataWriter
wqos.properties().properties().emplace_back(
    "fastdds.content_filtering_on_late_joiners",
    "true");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <data_writer profile_name="cft_on_late_joiners_xml_profile">
            <propertiesPolicy>
                <properties>
                    <!-- Enable content filtering on late joiners -->
                    <property>
                        <name>fastdds.content_filtering_on_late_joiners</name>
                        <value>true</value>
                    </property>
                </properties>
            </propertiesPolicy>
        </data_writer>
    </profiles>
</dds>
```
