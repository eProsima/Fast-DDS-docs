<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-zero-copy"></a>

<a id="zero-copy-communication"></a>

# Zero-Copy communication

This section explains how to configure a Zero-Copy communication in *Fast DDS*.
The Zero-Copy communication allows the transmission of data between applications
without copying data in memory, saving time and resources.
In order to achieve this, it uses Data-sharing delivery between the [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)
and the [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader), and data buffer loans between the application and *Fast DDS*.

<a id="overview"></a>

## Overview

[Data-sharing delivery](../../transport/datasharing.md#datasharing-delivery) provides a communication channel between a DataWriter and a DataReader
using shared memory. Therefore, it does not require copying the sample data to transmit it.

[DataWriter sample loaning](../../dds_layer/publisher/dataWriter/publishingData.md#dds-layer-publisher-write-loans)
is a *Fast DDS* extension that allows the application to borrow
a buffer for a sample in the publishing DataWriter.
The sample can be constructed directly on this buffer,
eliminating the need to copy it to the DataWriter afterwards.
This prevents the copying of the data between the publishing application and the DataWriter.
If Data-sharing delivery is used, the loaned data buffer will be in the shared memory itself.

Reading the data on the subscriber side can also be done
with [loans from the DataReader](../../dds_layer/subscriber/dataReader/readingData.md#dds-layer-subscriber-accessreceived-loans).
The application gets the received samples as a reference to the receive queue itself.
This prevents the copying of the data from the DataReader to the receiving application.
Again, if Data-sharing delivery is used, the loaned data will be in the shared memory,
and will indeed be the same memory buffer used in the DataWriter history.

Combining these three features, we can achieve Zero-Copy communication between the
publishing application and the subscribing application.

<a id="getting-started"></a>

## Getting started

To enable Zero-Copy perform the following steps:

1. Define a plain and bounded type in an IDL file and generate the corresponding source code for further processing
   with the [Fast DDS-Gen](../../../fastddsgen/introduction/introduction.md#fastddsgen-intro) tool.
   ```omg-idl
   @extensibility(FINAL)
   struct LoanableHelloWorld
   {
       unsigned long index;
       char message[256];
   };
   ```
2. On the DataWriter side:
   1. Create a DataWriter for the previous type. Make sure that the DataWriter does not have DataSharing disabled.
   2. Get a loan on a sample using [`loan_sample()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind).
   3. Write the sample using [`write()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv).
3. On the DataReader side:
   1. Create a DataReader for the previous type. Make sure that the DataReader does not have DataSharing disabled.
   2. Take/read samples using the available functions in the DataReader.
      Please refer to section [Loaning and Returning Data and SampleInfo Sequences](../../dds_layer/subscriber/dataReader/readingData.md#dds-layer-subscriber-accessreceived-loans) for further detail on how to access
      to loans of the received data.
   3. Return the loaned samples using [`DataReader::return_loan()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader11return_loanER18LoanableCollectionR13SampleInfoSeq).

<a id="writing-and-reading-in-zero-copy-transfers"></a>

## Writing and reading in Zero-Copy transfers

The following is an example of how to publish and receive samples with DataWriters and DataReaders respectively
that implement Zero-Copy.

<a id="datawriter"></a>

### DataWriter

When the DataWriter is created, *Fast DDS* will pre-allocate a pool of
[`max_samples`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) + [`extra_samples`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13extra_samplesE) samples that reside
in a shared memory mapped file.
This pool will be used to loan samples when the [`loan_sample()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind) function is called.

An application example of a DataWriter that supports Zero-Copy using the *Fast DDS* library is presented below.
There are several points to note in the following code:

* Not disabling the [DataSharingQosPolicy](../../dds_layer/core/policy/eprosimaExtensions.md#datasharingqospolicy).
  [`AUTO`](../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind4AUTOE) kind automatically enables Zero-Copy when possible.
* The use of the [`loan_sample()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind) function to access and modify data samples.
* The writing of data samples.

```c++
// CREATE THE PARTICIPANT
DomainParticipantQos pqos;
pqos.name("Participant_pub");
DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

// REGISTER THE TYPE
TypeSupport type(new LoanableHelloWorldPubSubType());
type.register_type(participant);

// CREATE THE PUBLISHER
Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

// CREATE THE TOPIC
Topic* topic = participant->create_topic(
    "LoanableHelloWorldTopic",
    type.get_type_name(),
    TOPIC_QOS_DEFAULT);

// CREATE THE WRITER
DataWriterQos wqos = publisher->get_default_datawriter_qos();
wqos.history().depth = 10;
wqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
// DataSharingQosPolicy has to be set to AUTO (the default) or ON to enable Zero-Copy
wqos.data_sharing().on("shared_directory");

DataWriter* writer = publisher->create_datawriter(topic, wqos);

std::cout << "LoanableHelloWorld DataWriter created." << std::endl;

int msgsent = 0;
void* sample = nullptr;
// Always call loan_sample() before writing a new sample.
// This function will provide the user with a pointer to an internal buffer where the data type can be
// prepared for sending.
if (RETCODE_OK == writer->loan_sample(sample))
{
    // Modify the sample data
    LoanableHelloWorld* data = static_cast<LoanableHelloWorld*>(sample);
    data->index() = msgsent + 1;
    memcpy(data->message().data(), "LoanableHelloWorld ", 20);

    std::cout << "Sending sample (count=" << msgsent
              << ") at address " << &data << std::endl
              << "  index=" << data->index() << std::endl
              << "  message=" << data->message().data() << std::endl;

    // Write the sample.
    // After this function returns, the middleware owns the sample.
    writer->write(sample);
}
```

<a id="datareader"></a>

### DataReader

The following is an application example of a DataReader that supports Zero-Copy using the *Fast DDS* library.
As shown in this code snippet, the configuration in the DataReader is similar to the DataWriter.
Be sure not to disable the [DataSharingQosPolicy](../../dds_layer/core/policy/eprosimaExtensions.md#datasharingqospolicy).
[`AUTO`](../../api_reference/dds_pim/core/policy/datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKind4AUTOE) kind automatically enables Zero-Copy when possible.

```c++
// CREATE THE PARTICIPANT
DomainParticipantQos pqos;
pqos.name("Participant_sub");
DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

// REGISTER THE TYPE
TypeSupport type(new LoanableHelloWorldPubSubType());
type.register_type(participant);

// CREATE THE SUBSCRIBER
Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

// CREATE THE TOPIC
Topic* topic = participant->create_topic(
    "LoanableHelloWorldTopic",
    type.get_type_name(),
    TOPIC_QOS_DEFAULT);

// CREATE THE READER
DataReaderQos rqos = subscriber->get_default_datareader_qos();
rqos.history().depth = 10;
rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
// DataSharingQosPolicy has to be set to AUTO (the default) or ON to enable Zero-Copy
rqos.data_sharing().automatic();

DataReader* reader = subscriber->create_datareader(topic, rqos, &datareader_listener);
```

Finally, the code snippet below implements the [`on_data_available()`](../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader) [DataReaderListener](../../dds_layer/subscriber/dataReaderListener/dataReaderListener.md#dds-layer-subscriber-datareaderlistener)
callback.
The key points to be noted in this function are:

* The declaration and handling of [`LoanableSequence`](../../api_reference/dds_pim/core/loanablesequence.md#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE).
* Checking [`DataReader::is_sample_valid()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader15is_sample_validEPKvPK10SampleInfo) for verifying that the sample was not replaced.
  Refer to [DataReader and DataWriter history coupling](../../transport/datasharing.md#datareader-datawriter-history-coupling) for further information.
* The use of the [`DataReader::return_loan()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader11return_loanER18LoanableCollectionR13SampleInfoSeq) function to indicate to the DataReader that the application has
  finished accessing the sequence.

```c++
void on_data_available(
        eprosima::fastdds::dds::DataReader* reader) override
{
    // Declare a LoanableSequence for a data type
    FASTDDS_SEQUENCE(DataSeq, LoanableHelloWorld);

    DataSeq data;
    SampleInfoSeq infos;
    // Access to the collection of data-samples and its corresponding collection of SampleInfo structures
    while (RETCODE_OK == reader->take(data, infos))
    {
        // Iterate over each LoanableCollection in the SampleInfo sequence
        for (LoanableCollection::size_type i = 0; i < infos.length(); ++i)
        {
            // Check whether the DataSample contains data or is only used to communicate of a
            // change in the instance
            if (infos[i].valid_data)
            {
                // Print the data.
                const LoanableHelloWorld& sample = data[i];

                ++samples;
                std::cout << "Sample received (count=" << samples
                          << ") at address " << &sample
                          << (reader->is_sample_valid(&sample,
                &infos[i]) ? " is valid" : " was replaced" ) << std::endl
                          << "  index=" << sample.index() << std::endl
                          << "  message=" << sample.message().data() << std::endl;
            }
        }
        // Indicate to the DataReader that the application is done accessing the collection of
        // data values and SampleInfo, obtained by some earlier invocation of read or take on the
        // DataReader.
        reader->return_loan(data, infos);
    }
}

```

<a id="caveats"></a>

## Caveats

* After calling [`write()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter5writeEPCKv), *Fast DDS* takes ownership of the sample and therefore it is no longer
  safe to make changes to that sample.
* If function [`loan_sample()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind) is called first and the sample is never written, it is necessary to use
  function [`discard_loan()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter12discard_loanERPv) to return the sample to the DataWriter.
  If this is not done, the subsequent calls to [`loan_sample()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter11loan_sampleERPv22LoanInitializationKind) may fail if DataWriter has no more
  [`extra_samples`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13extra_samplesE) to loan.
* The current maximum supported sample size is the maximum value of an `uint32_t`.

<a id="constraints"></a>

## Constraints

Although Zero-Copy can be used for one or several *Fast DDS* application processes running on the same machine,
it has some constraints:

* Only plain types are supported.

  A plain type is a type whose CDR representation matches its in-memory representation.
  This requirement avoids the copy between the CDR buffer and the user buffer because the data representation is the
  same.
  Consequently, only [primitive types](../../../fastddsgen/dataTypes/dataTypes.md#idl-primitive-types) (except `string`), arrays of these primitive
  types, and structures with FINAL [extensibility](../../../fastddsgen/dataTypes/dataTypes.md#extensibility) and members of these primitive types,
  are considered to be plain (Fast DDS also provides an API to check if a defined type is plain:
  [`TypeSupport::is_plain()`](../../api_reference/dds_pim/topic/typesupport.md#_CPPv4NK8eprosima7fastdds3dds11TypeSupport8is_plainE22DataRepresentationId_t)).
* Constraints for [datasharing delivery](../../transport/datasharing.md#datasharing-delivery-constraints) also apply.

#### NOTE
Zero-Copy transfer support for non-plain types may be implemented in future releases of *Fast DDS*.

<a id="next-steps"></a>

## Next steps

A hello world example suitable for supported delivery mechanisms can be found in the
[delivery_mechanisms folder](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/delivery_mechanisms).
It shows a publisher and a subscriber that communicate through the desired delivery mechanism.
As long as it is zero-copy compatible, both intra-process and data-sharing delivery mechanisms are zero-copy if used.
