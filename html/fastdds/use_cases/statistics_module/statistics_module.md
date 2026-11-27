<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="statistics-module"></a>

<a id="id1"></a>

# Statistics module

*eProsima Fast DDS* [Statistics Module](../../statistics/statistics.md#statistics) allows the user to monitor the data being exchanged by its application.
In order to use this module, the user must enable it in the monitored application, and create another application that
receives the data being published by the statistics DataWriters.
The user can also use for the latter the
[eProsima Fast DDS Statistics Backend](https://fast-dds-statistics-backend.readthedocs.io/en/latest/) which already
implements the collection and aggregation of the data coming from the statistics topics.

<a id="enable-statistics-module"></a>

## Enable Statistics module

In *Fast DDS*, the Statistics module has to be enabled both at build and runtime.
On the one hand, [CMake option](../../../installation/configuration/cmake_options.md#cmake-options) `FASTDDS_STATISTICS` must be enabled when building the library
(since Fast DDS v2.9.0 this CMake option is enabled by default).
**[Pro]** In *Fast DDS* **[Pro]**, the Statistics module is always built, so it only has to be enabled at runtime.
On the other hand, the desired statistics DataWriters should be enabled using the [Statistics Module DDS Layer](../../statistics/dds_layer/statistics_dds_layer.md#statistics-dds-layer).

The statistics DataWriters can be enabled automatically using the [PropertyPolicyQos](../../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos) `fastdds.statistics` and
the [FASTDDS_STATISTICS](../../env_vars/env_vars.md#env-vars-fastdds-statistics) environment variable.
They can also be enabled manually following the next example:

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

<a id="create-monitoring-application"></a>

## Create monitoring application

Once the monitored application is publishing the collected data within the statistics topics enabled by the user,
another application should be configured to subscribe to those topics.
This application is a DDS standard application where the statistics DataReaders should be created.
In order to create these statistics DataReaders, the user should follow the next steps:

* Using the [statistics IDL](https://github.com/eProsima/Fast-DDS/blob/master/include/fastdds/statistics/types.idl)
  provided in the public API, generate the [`TopicDataTypes`](../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) with [Fast DDS-Gen](../../../fastddsgen/usage/usage.md#fastddsgen-usage).
* Create the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) and register the [`TopicDataTypes`](../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) and the corresponding statistics
  [`Topics`](../../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE).
* Create the statistics DataReaders using the corresponding statistics topic.
