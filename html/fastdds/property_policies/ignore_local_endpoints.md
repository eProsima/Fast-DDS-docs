<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="property-ignore-local-endpoints"></a>

<a id="ignore-local-endpoints"></a>

# Ignore Local Endpoints

By default, Fast DDS will automatically match all the endpoints (meaning [DataReaders](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriters](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)) belonging to a
given [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) as soon as they share the same [Topic](../dds_layer/topic/topic.md#dds-layer-topic) and have compatible Qos.
This however can result in undesired feedback whenever an application creates a [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and a [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) under
the same [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) on a shared [Topic](../dds_layer/topic/topic.md#dds-layer-topic).
Although this feedback can be filtered out at the application level upon data reception by filtering out messages coming
from a [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) belonging to the same [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) on the [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) receiving the data (by looking at the
[`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE)), this entails for a data sample to go all the way to the [DataReaderListener](../dds_layer/subscriber/dataReaderListener/dataReaderListener.md#dds-layer-subscriber-datareaderlistener) just to be discarded
by an overcomplicated application business logic.
For this reason, Fast DDS offers the possibility of instructing the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) to avoid the matching of local
endpoints through the following property:

| PropertyPolicyQos name             | PropertyPolicyQos value   | Default value   |
|------------------------------------|---------------------------|-----------------|
| `"fastdds.ignore_local_endpoints"` | `"true"`/`"false"`        | `"false"`       |

### C++

```c++
DomainParticipantQos participant_qos;

// Avoid local matching of this participant's endpoints
participant_qos.properties().properties().emplace_back(
    "fastdds.ignore_local_endpoints",
    "true");
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="ignore_local_endpoints_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <!-- Avoid local matching of this participant's endpoints -->
                        <property>
                            <name>fastdds.ignore_local_endpoints</name>
                            <value>true</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
    </profiles>
</dds>
```

#### NOTE
An invalid value of `fastdds.ignore_local_endpoints` results in the default behaviour.
