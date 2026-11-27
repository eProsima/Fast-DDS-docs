<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher-write"></a>

<a id="publishing-data"></a>

# Publishing data

The user informs of a change in the value of a data instance with the [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) member function on the
[DataWriter](dataWriter.md#dds-layer-publisher-datawriter). This change will then be communicated to every
[DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) matched with the DataWriter.
As a side effect, this operation asserts liveliness on the DataWriter itself,
the [Publisher](../publisher/publisher.md#dds-layer-publisher-publisher) and the [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).

The function takes two arguments:

* A pointer to the data instance with the new values.
* The handler to the instance.

An empty (i.e., default constructed [`InstanceHandle_t`](../../../api_reference/rtps/common/InstanceHandle/InstanceHandle_t.md#_CPPv4N8eprosima7fastdds4rtps16InstanceHandle_tE)) instance handler can be used for the argument handle.
This indicates that the identity of the instance should be automatically deduced from the key of the
instance data.
Alternatively, the member function [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) is overloaded to take only the pointer to the data instance,
which will always deduced the identity from the key of the instance data.

If the handle is not empty, then it must correspond to the value obtained with the `getKey()` of the
[`TypeSupport`](../../../api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) instance.
Otherwise the write function will fail with `RETCODE_PRECONDITION_NOT_MET`.

```c++
// Register the data type in the DomainParticipant.
TypeSupport custom_type_support(new CustomDataType());
custom_type_support.register_type(participant, custom_type_support.get_type_name());

// Create a Topic with the registered type.
Topic* custom_topic =
        participant->create_topic("topic_name", custom_type_support.get_type_name(), TOPIC_QOS_DEFAULT);
if (nullptr == custom_topic)
{
    // Error
    return;
}

// Create a DataWriter
DataWriter* data_writer =
        publisher->create_datawriter(custom_topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == data_writer)
{
    // Error
    return;
}

// Get a data instance
void* data = custom_type_support->create_data();

// Fill the data values
// (...)

// Publish the new value, deduce the instance handle
if (data_writer->write(data, eprosima::fastdds::rtps::InstanceHandle_t()) != RETCODE_OK)
{
    // Error
    return;
}

// The data instance can be reused to publish new values,
// but delete it at the end to avoid leaks
custom_type_support->delete_data(data);
```

<a id="dds-layer-publisher-write-blocking"></a>

<a id="blocking-of-the-write-operation"></a>

## Blocking of the write operation

If the reliability kind is set to `RELIABLE` on the [DataWriterQos](dataWriter.md#dds-layer-publisher-datawriterqos),
the [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) operation may block.
Specifically, if the limits specified in the configured resource limits have been reached, the
[`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) operation will block waiting for space to become available.
Under these circumstances, the reliability `max_blocking_time` configures the maximum time
the write operation may block waiting.
If `max_blocking_time` elapses before the DataWriter is able to store
the modification without exceeding the limits, the write operation will fail and return `TIMEOUT`.

<a id="dds-layer-publisher-write-loans"></a>

<a id="borrowing-a-data-buffer"></a>

## Borrowing a data buffer

When the user calls [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) with a new sample value,
the data is copied from the given sample to the DataWriter’s memory.
For large data types this copy can consume significant time and memory resources.
Instead, the DataWriter can loan a sample from its memory to the user,
and the user can fill this sample with the required values.
When [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) is called with such a loaned sample,
the DataWriter does not copy its contents, as it already owns the buffer.

To use loaned data samples in publications, perform the following steps:

1. Get a reference to a loaned sample using [`loan_sample()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind).
2. Use the reference to build the data sample.
3. Write the sample using [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv).

Once [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) has been called with a loaned sample,
the loan is considered returned, and it is not safe to make any
changes on the contents of the sample.

If function [`loan_sample()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind) is called but the sample is never written,
the loan must be returned to the DataWriter using [`discard_loan()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter12discard_loanERPv).
Otherwise the DataWriter may run out of samples.

```c++
    // Borrow a data instance
    void* data = nullptr;
    if (RETCODE_OK == data_writer->loan_sample(data))
    {
        bool error = false;

        // Fill the data values
        // (...)

        if (error)
        {
            // Return the loan without publishing
            data_writer->discard_loan(data);
            return;
        }

        // Publish the new value
        if (data_writer->write(data, eprosima::fastdds::rtps::InstanceHandle_t()) != RETCODE_OK)
        {
            // Error
            return;
        }
    }
```

<a id="dds-layer-publisher-prefiltering"></a>

<a id="prefiltering-out-datareaders"></a>

## Prefiltering out DataReaders

The user can use an overload of the [`write()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv) method that allows providing a [`WriteParams`](../../../api_reference/rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) structure.
One of the members of this latter structure is the [`UserWriteData`](../../../api_reference/rtps/common/WriteParams/WriteParams.md#_CPPv4N8eprosima7fastdds4rtps11WriteParams13UserWriteDataE) in which the user
can store extra information to be used by the prefiltering mechanism and it can be set using the
[`user_write_data()`](../../../api_reference/rtps/common/WriteParams/WriteParams.md#_CPPv4NK8eprosima7fastdds4rtps11WriteParams15user_write_dataEv).

By implementing the [`IContentFilter`](../../../api_reference/dds_pim/topic/icontentfilter.md#_CPPv4N8eprosima7fastdds3dds14IContentFilterE) and passing it to the DataWriter with [`DataWriter::set_sample_prefilter()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter20set_sample_prefilterENSt10shared_ptrI14IContentFilterEE),
user can prevent the DataWriter from sending samples to the matched DataReaders based on both
the content of the sample ([`SerializedPayload_t`](../../../api_reference/rtps/common/SerializedPayload/SerializedPayload_t.md#_CPPv4N8eprosima7fastdds4rtps19SerializedPayload_tE)) and/or the `user_write_data` within
`FilteredSampleInfo`.
If the return value of [`evaluate()`](../../../api_reference/dds_pim/topic/icontentfilter.md#_CPPv4NK8eprosima7fastdds3dds14IContentFilter8evaluateERK17SerializedPayloadRK16FilterSampleInfoRK6GUID_t) is `false`, the sample will not be sent to the DataReader
identified by its [`Guid_t`](../../../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) in the method’s input argument.
It is strongly recommended that the [`evaluate()`](../../../api_reference/dds_pim/topic/icontentfilter.md#_CPPv4NK8eprosima7fastdds3dds14IContentFilter8evaluateERK17SerializedPayloadRK16FilterSampleInfoRK6GUID_t) implementation neither performs any blocking
operation nor uses the DataWriter, as this may lead to deadlocks or undesired behavior.

#### WARNING
Prefiltering is currently incompatible with [DataSharingQosPolicy](../../core/policy/eprosimaExtensions.md#datasharingqospolicy).
Hence, ensure that the [`kind()`](../../../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4NK8eprosima7fastdds3dds20DataSharingQosPolicy4kindEv) is set to `OFF` or
if `AUTO` is used, [constraints](../../../transport/datasharing.md#datasharing-delivery-constraints) are not met.

Next is an example of how to use the prefiltering mechanism:

```c++
struct CustomUserWriteData : public eprosima::fastdds::rtps::WriteParams::UserWriteData
{
    CustomUserWriteData(
            const eprosima::fastdds::rtps::GuidPrefix_t& prefix)
        : filtered_out_prefix(prefix)
    {
    }

    eprosima::fastdds::rtps::GuidPrefix_t filtered_out_prefix;
};

struct CustomPreFilter : public eprosima::fastdds::dds::IContentFilter
{
    // Custom evaluation
    bool evaluate(
            const SerializedPayload& payload,
            const FilterSampleInfo& filter_sample_info,
            const eprosima::fastdds::rtps::GUID_t& reader_guid) const override
    {
        static_cast<void>(payload);
        static_cast<void>(filter_sample_info);

        bool sample_should_be_sent = true;

        auto custom_write_data =
                std::static_pointer_cast<CustomUserWriteData>(
            filter_sample_info.user_write_data);

        // If the reader is the one to filter out, do not send the sample
        if (custom_write_data->filtered_out_prefix == reader_guid.guidPrefix)
        {
            sample_should_be_sent = false;
        }
        return sample_should_be_sent;
    }

};

// Create a DataWriter
DataWriter* data_writer =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == data_writer)
{
    // Error
    return;
}

// Set a prefilter on the filtered reader
eprosima::fastdds::dds::ReturnCode_t ret = data_writer->set_sample_prefilter(
    std::make_shared<CustomPreFilter>());

if (ret != eprosima::fastdds::dds::RETCODE_OK)
{
    // Error
    return;
}

// Create a custom user write data
eprosima::fastdds::rtps::GuidPrefix_t filtered_out_prefix;
std::istringstream("44.53.00.5f.45.60.52.4f.53.49.7c.41") >> filtered_out_prefix;

std::shared_ptr<CustomUserWriteData> custom_write_data =
        std::make_shared<CustomUserWriteData>(filtered_out_prefix);

// Set the custom user write data
eprosima::fastdds::rtps::WriteParams write_params;
write_params.user_write_data(custom_write_data);

HelloWorld sample;     //Auto-generated container class for topic data from Fast DDS-Gen
sample.msg("Hello there!");     // Add contents to the message

// Write a sample with the custom user write data
data_writer->write(&sample, write_params);

```
