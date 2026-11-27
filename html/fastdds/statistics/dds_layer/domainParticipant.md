<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="statistics-domainparticipant"></a>

<a id="statistics-domain-participant"></a>

# Statistics Domain Participant

In order to start collecting data in one of the statistics topics ([Statistics Topic names](topic_names.md#statistics-topic-names)), the corresponding
statistics DataWriter should be enabled.
In fact, *Fast DDS Statistics module* can be enabled and disabled at runtime.
For this purpose, *Fast DDS Statistics module* exposes an extended DDS `statistics::dds::DomainParticipant` API:

<a id="statistics-enable-datawriters"></a>

<a id="enable-statistics-datawriters"></a>

## Enable statistics DataWriters

Statistics DataWriters can be enabled in different ways.
It can be done automatically (see [Automatically enabling statistics DataWriters](#auto-enabling-statistics-datawriters)).
Alternatively, Statistics DataWriters can be enabled at run time using one of two methods:
[`enable_statistics_datawriter()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant28enable_statistics_datawriterERKNSt6stringERK13DataWriterQos) or [`enable_statistics_datawriter_with_profile()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant41enable_statistics_datawriter_with_profileERKNSt6stringERKNSt6stringE).

[`enable_statistics_datawriter()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant28enable_statistics_datawriterERKNSt6stringERK13DataWriterQos) method requires as parameters:

* Name of the statistics topic to be enabled (see [Statistics Topic names](topic_names.md#statistics-topic-names) for the statistics topic list).
* DataWriter QoS profile (see [Statistics DataWriter recommended QoS](qos.md#statistics-datawriter-qos) for the recommended profile).

It is possible to define specific desired QoS through DataWriter profile on the FASTDDS_DEFAULT_PROFILES_FILE
(see [XML profiles](../../xml_configuration/xml_configuration.md#xml-profiles)).
[`enable_statistics_datawriter_with_profile()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant41enable_statistics_datawriter_with_profileERKNSt6stringERKNSt6stringE) method enables a DataWriter by searching a specific DataWriter XML profile.
On those profiles, specific QoS can be set.

[`enable_statistics_datawriter_with_profile()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant41enable_statistics_datawriter_with_profileERKNSt6stringERKNSt6stringE) method requires as parameters:

* Name of the XML profile to use to fill the QoS structure of the DataWriter.
* Name of the statistics topic name to be enabled. (see [Statistics Topic names](topic_names.md#statistics-topic-names) for the statistics topic list).

<a id="disable-statistics-datawriters"></a>

## Disable statistics DataWriters

Statistics DataWriters are disabled using the method [`disable_statistics_datawriter()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant29disable_statistics_datawriterERKNSt6stringE).
This method requires as parameter:

* Name of the statistics topic to be disabled (see [Statistics Topic names](topic_names.md#statistics-topic-names) for the statistics topic list).

<a id="example"></a>

## Example

<a id="obtain-pointer-to-the-extended-statisticsdomainparticipant-api-class"></a>

## Obtain pointer to the extended `statistics::dds::DomainParticipant` class

The [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) is created using the [`create_participant()`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask) provided by the
[`DomainParticipantFactory`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE).
This method returns a pointer to the DDS standard DomainParticipant created.
In order to obtain the pointer to the child `statistics::dds::DomainParticipant` which extends the DDS API, the
`static` method [`narrow()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant6narrowEP17DomainParticipant) is provided.

#### NOTE
**[Pro]** In *Fast DDS* **[Pro]**, the statistics API is part of the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) itself, so it can be called
directly on the pointer returned by [`create_participant()`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask).
`statistics::dds::DomainParticipant` is kept as an alias of [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), and [`narrow()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant6narrowEP17DomainParticipant) returns
the participant it is given, so code written for *Fast DDS* keeps working.

The following example shows how to use the Statistics module extended DDS API:

```c++
// Create a DomainParticipant
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Enable statistics DataWriter
if (participant->enable_statistics_datawriter(eprosima::fastdds::statistics::GAP_COUNT_TOPIC,
        eprosima::fastdds::statistics::dds::STATISTICS_DATAWRITER_QOS) != RETCODE_OK)
{
    // Error
    return;
}

// Use the DomainParticipant to communicate
// (...)

// Disable statistics DataWriter
if (participant->disable_statistics_datawriter(eprosima::fastdds::statistics::GAP_COUNT_TOPIC) !=
        RETCODE_OK)
{
    // Error
    return;
}

// Delete DomainParticipant
if (DomainParticipantFactory::get_instance()->delete_participant(participant) != RETCODE_OK)
{
    // Error
    return;
}
```

<a id="auto-enabling-statistics-datawriters"></a>

<a id="automatically-enabling-statistics-datawriters"></a>

## Automatically enabling statistics DataWriters

The statistics DataWriters can be directly enabled using the [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos)
[`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv) `fastdds.statistics`.
The value of this property is a semicolon separated list containing the
[statistics topic name aliases](topic_names.md#statistics-topic-names) of those DataWriters that the user wants to enable.
The property can be set either programmatically or loading an XML file.
If the property is set in both ways, the priority would depend on the API and the QoS profile provided:

* XML settings have priority if [`create_participant_with_profile()`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory31create_participant_with_profileE10DomainId_tRKNSt6stringEP25DomainParticipantListenerRK10StatusMask) is called with a valid
  participant profile.
* XML settings also have priority if [`create_participant()`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask) is called using
  [`PARTICIPANT_QOS_DEFAULT`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds23PARTICIPANT_QOS_DEFAULTE) and a participant profile exists in the XML file with the `is_default_profile` option
  set to `true` ([DomainParticipant XML attributes](../../xml_configuration/domainparticipant.md#domainparticipantattributes)).
* The property set programmatically is used only when [`create_participant()`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory18create_participantE10DomainId_tRK20DomainParticipantQosP25DomainParticipantListenerRK10StatusMask) is called with
  the specific QoS.

Another way of enabling statistics DataWriters, compatible with the previous one, is setting the
[FASTDDS_STATISTICS](../../env_vars/env_vars.md#env-vars-fastdds-statistics) environment variable.
The statistics DataWriters that will be enabled when the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) is enabled would be the union between
those specified in the [`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv) `fastdds.statistics` and those included with the
environment variable.

The following examples show how to use all the previous methods:

### C++

```c++
DomainParticipantQos pqos;

// Activate Fast DDS Statistics module
pqos.properties().properties().emplace_back("fastdds.statistics",
        "HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;PDP_PACKETS_TOPIC");
```

### XML

```xml
<participant profile_name="statistics_domainparticipant_conf_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <!-- Activate Fast DDS Statistics Module -->
                <property>
                    <name>fastdds.statistics</name>
                    <value>HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;PDP_PACKETS_TOPIC</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
```

### Environment Variable Linux

```bash
export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;PDP_PACKETS_TOPIC"
```

### Environment Variable Windows

```bash
set FASTDDS_STATISTICS=HISTORY_LATENCY_TOPIC;ACKNACK_COUNT_TOPIC;PDP_PACKETS_TOPIC
```

#### NOTE
These are all the statistics topics:

```bash
HISTORY_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;SUBSCRIPTION_THROUGHPUT_TOPIC;HEARTBEAT_COUNT_TOPIC;ACKNACK_COUNT_TOPIC;NACKFRAG_COUNT_TOPIC;GAP_COUNT_TOPIC;DATA_COUNT_TOPIC;RESENT_DATAS_TOPIC;PDP_PACKETS_TOPIC;EDP_PACKETS_TOPIC
```

#### NOTE
Be aware that automatically enabling the statistics DataWriters using all these methods implies using the recommended
QoS profile [`STATISTICS_DATAWRITER_QOS`](../../api_reference/statistics/datawriter_qos.md#_CPPv4N8eprosima7fastdds10statistics3dds25STATISTICS_DATAWRITER_QOSE). For more information, please refer to [Statistics DataWriter recommended QoS](qos.md#statistics-datawriter-qos).
However, if an XML profile is defined, the QoS applied are those defined in the profile,
and for those QoS that are not specified in that profile, the default library QoS are applied
(see [DataWriterQos](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos) for the standard eProsima’s DataWriter QoS),
and not the recommended QoS for the Statistics DataWriters.

For the creation of an automatically enabled Datawriter, the priority for setting its QoS is the following:

* First, if a specific profile exists for the statistics topic, that one is applied.
* If that is not the case but a generic profile for statistics DataWriters exists, that one is applied.
* If no profile is defined in XML file, the recommended statistics QoS are applied.

#### NOTE
The generic DataWriter profile defined in the FASTDDS_DEFAULT_PROFILES_FILE XML needs to be named as
GENERIC_STATISTICS_PROFILE.

The specific DataWriter profile defined in the FASTDDS_DEFAULT_PROFILES_FILE XML needs to be named using the
same statistic topic alias or name (see [Statistics Topic names](topic_names.md#statistics-topic-names) for the alias corresponding to each statistic
topic) that has been used in the [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) [`properties()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv) `fastdds.statistics`
(see [Statistics Module Settings](../../property_policies/non_consolidated_qos.md#property-policies-statistics)) or the [FASTDDS_STATISTICS](../../env_vars/env_vars.md#env-vars-fastdds-statistics) environment variable,
where the enabling of the corresponding statistics
topic has been set.
