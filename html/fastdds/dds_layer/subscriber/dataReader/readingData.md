<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-accessreceived"></a>

<a id="accessing-received-data"></a>

# Accessing received data

The application can access and consume the data values received on the [DataReader](dataReader.md#dds-layer-subscriber-datareader)
by *reading* or *taking*.

* **Reading** is done with any of the following member functions:
  * [`DataReader::read_next_sample()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16read_next_sampleEPvP10SampleInfo) reads the next, non-previously accessed data value available
    on the DataReader, and stores it in the provided data buffer.
  * [`DataReader::read()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4readER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask), [`DataReader::read_instance()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader13read_instanceER18LoanableCollectionR13SampleInfoSeqNSt7int32_tERK16InstanceHandle_t15SampleStateMask13ViewStateMask17InstanceStateMask), and [`DataReader::read_next_instance()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader18read_next_instanceER18LoanableCollectionR13SampleInfoSeqNSt7int32_tERK16InstanceHandle_t15SampleStateMask13ViewStateMask17InstanceStateMask)
    provide mechanisms to get a collection of samples matching certain conditions.
* **Taking** is done with any of the following member functions:
  * [`DataReader::take_next_sample()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16take_next_sampleEPvP10SampleInfo) reads the next, non-previously accessed data value available on the DataReader,
    and stores it in the provided data buffer.
  * [`DataReader::take()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4takeER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask), [`DataReader::take_instance()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader13take_instanceER18LoanableCollectionR13SampleInfoSeqNSt7int32_tERK16InstanceHandle_t15SampleStateMask13ViewStateMask17InstanceStateMask), and [`DataReader::take_next_instance()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader18take_next_instanceER18LoanableCollectionR13SampleInfoSeqNSt7int32_tERK16InstanceHandle_t15SampleStateMask13ViewStateMask17InstanceStateMask)
    provide mechanisms to get a collection of samples matching certain conditions.

  When taking data, the returned samples are also removed from the DataReader, so they are no longer accessible.

When there is no data in the DataReader matching the required conditions, all the operations will return
`NO_DATA` and output parameter will remain unchanged.

In addition to the data values, the data access operations also provide SampleInfo instances with additional
information that help interpreting the returned data values, like the originating
[DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) or the publication time stamp.
Please, refer to the [SampleInfo](../sampleInfo/sampleInfo.md#dds-layer-subscriber-sampleinfo) section for an extensive description of its contents.

<a id="dds-layer-subscriber-accessreceived-loans"></a>

<a id="loaning-and-returning-data-and-sampleinfo-sequences"></a>

## Loaning and Returning Data and SampleInfo Sequences

The [`DataReader::read()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4readER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) and [`DataReader::take()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4takeER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) operations (and their variants) return information to the
application in two sequences:

* Received DDS data samples in a sequence of the data type
* Corresponding information about each DDS sample in a SampleInfo sequence

These sequences are parameters that are passed by the application code into the
[`DataReader::read()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4readER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) and [`DataReader::take()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4takeER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) operations.
When the passed sequences are empty (they are initialized but have a maximum length of 0), the middleware will
fill those sequences with memory directly loaned from the receive queue itself.
There is no copying of the data or SampleInfo when the contents of the sequences are loaned.
This is certainly the most efficient way for the application code to retrieve the data.

When doing so, however, the code must return the loaned sequences back to the middleware, so that they can be reused
by the receive queue.
If the application does not return the loan by calling the [`DataReader::return_loan()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader11return_loanER18LoanableCollectionR13SampleInfoSeq) operation, then Fast DDS
will eventually run out of memory to store DDS data samples received from the network for that DataReader.
See the code below for an example of borrowing and returning loaned sequences.

```c++
    // Sequences are automatically initialized to be empty (maximum == 0)
    FooSeq data_seq;
    SampleInfoSeq info_seq;

    // with empty sequences, a take() or read() will return loaned
    // sequence elements
    ReturnCode_t ret_code = data_reader->take(data_seq, info_seq,
                    LENGTH_UNLIMITED, ANY_SAMPLE_STATE,
                    ANY_VIEW_STATE, ANY_INSTANCE_STATE);

    // process the returned data

    // must return the loaned sequences when done processing
    data_reader->return_loan(data_seq, info_seq);
```

<a id="dds-layer-subscriber-accessreceived-data"></a>

<a id="processing-returned-data"></a>

## Processing returned data

After calling the [`DataReader::read()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4readER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) or [`DataReader::take()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4takeER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) operations, accessing the data on the returned
sequences is quite easy.
The sequences API provides a **length()** operation returning the number of elements in the collections.
The application code just needs to check this value and use the **[]** operator to access the corresponding elements.
Elements on the DDS data sequence should only be accessed when the corresponding element on the SampleInfo sequence
indicate that valid data is present.
When using Data Sharing, it is also important to check that the sample is valid (i.e, not replaced,
refer to [DataReader and DataWriter history coupling](../../../transport/datasharing.md#datareader-datawriter-history-coupling) for further information in this regard).

```c++
    // Sequences are automatically initialized to be empty (maximum == 0)
    FooSeq data_seq;
    SampleInfoSeq info_seq;

    // with empty sequences, a take() or read() will return loaned
    // sequence elements
    ReturnCode_t ret_code = data_reader->take(data_seq, info_seq,
                    LENGTH_UNLIMITED, ANY_SAMPLE_STATE,
                    ANY_VIEW_STATE, ANY_INSTANCE_STATE);

    // process the returned data
    if (ret_code == RETCODE_OK)
    {
        // Both info_seq.length() and data_seq.length() will have the number of samples returned
        for (FooSeq::size_type n = 0; n < info_seq.length(); ++n)
        {
            // Only samples with valid data should be accessed
            if (info_seq[n].valid_data && data_reader->is_sample_valid(&data_seq[n], &info_seq[n]))
            {
                // Do something with data_seq[n]
            }
        }

        // must return the loaned sequences when done processing
        data_reader->return_loan(data_seq, info_seq);
    }
```

<a id="dds-layer-subscriber-accessreceived-listener"></a>

<a id="accessing-data-on-callbacks"></a>

## Accessing data on callbacks

When the DataReader receives new data values from any matching DataWriter, it informs the application through
two Listener callbacks:

* [`on_data_available()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader).
* [`on_data_on_readers()`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListener18on_data_on_readersEP10Subscriber).

These callbacks can be used to retrieve the newly arrived data, as in the following example.

```c++
class CustomizedDataReaderListener : public DataReaderListener
{

public:

    CustomizedDataReaderListener()
        : DataReaderListener()
    {
    }

    virtual ~CustomizedDataReaderListener()
    {
    }

    void on_data_available(
            DataReader* reader) override
    {
        // Create a data and SampleInfo instance
        Foo data;
        SampleInfo info;

        // Keep taking data until there is nothing to take
        while (reader->take_next_sample(&data, &info) == RETCODE_OK)
        {
            if (info.valid_data)
            {
                // Do something with the data
                std::cout << "Received new data value for topic "
                          << reader->get_topicdescription()->get_name()
                          << std::endl;
            }
            else
            {
                std::cout << "Remote writer for topic "
                          << reader->get_topicdescription()->get_name()
                          << " is dead" << std::endl;
            }
        }
    }

};
```

#### NOTE
If several new data changes are received at once, the callbacks may be triggered just once,
instead of once per change.
The application must keep *reading* or *taking* until no new changes are available.

<a id="dds-layer-subscriber-accessreceived-wait"></a>

<a id="accessing-data-with-a-waiting-thread"></a>

## Accessing data with a waiting thread

<a id="wait-sets-and-dataavailable-status-condition"></a>

### Wait-sets and DataAvailable status condition

Instead of relying on the Listener to try and get new data values,
the application can also dedicate a thread to wait until any new data is available on the
DataReader.
This can be done using a wait-set to wait for a change on the DataAvailable status.

```c++
// Create a DataReader
DataReader* data_reader =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == data_reader)
{
    // Error
    return;
}

// Prepare a wait-set to wait for data on the DataReader
WaitSet wait_set;
StatusCondition& condition = data_reader->get_statuscondition();
condition.set_enabled_statuses(StatusMask::data_available());
wait_set.attach_condition(condition);

// Create a data and SampleInfo instance
Foo data;
SampleInfo info;

//Define a timeout of 5 seconds
eprosima::fastdds::dds::Duration_t timeout (5, 0);

// Loop reading data as it arrives
// This will make the current thread to be dedicated exclusively to
// waiting and reading data until the remote DataWriter dies
while (true)
{
    ConditionSeq active_conditions;
    if (RETCODE_OK == wait_set.wait(active_conditions, timeout))
    {
        while (RETCODE_OK == data_reader->take_next_sample(&data, &info))
        {
            if (info.valid_data)
            {
                // Do something with the data
                std::cout << "Received new data value for topic "
                          << topic->get_name()
                          << std::endl;
            }
            else
            {
                // If the remote writer is not alive, we exit the reading loop
                std::cout << "Remote writer for topic "
                          << topic->get_name()
                          << " is dead" << std::endl;
                break;
            }
        }
    }
    else
    {
        std::cout << "No data this time" << std::endl;
    }
}
```

<a id="datareader-non-blocking-calls"></a>

### DataReader non-blocking calls

The same could be achieved using the `DataReader::wait_for_unread_message()` member function,
that blocks until a new data sample is available or the given timeout expires.
If no new data was available after the timeout expired, it will return with value `false`.
This function returning with value `true` means there is new data available on the
[DataReader](dataReader.md#dds-layer-subscriber-datareader) ready for the application to retrieve.

```c++
// Create a DataReader
DataReader* data_reader =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == data_reader)
{
    // Error
    return;
}

// Create a data and SampleInfo instance
Foo data;
SampleInfo info;

//Define a timeout of 5 seconds
eprosima::fastdds::dds::Duration_t timeout (5, 0);

// Loop reading data as it arrives
// This will make the current thread to be dedicated exclusively to
// waiting and reading data until the remote DataWriter dies
while (true)
{
    if (data_reader->wait_for_unread_message(timeout))
    {
        if (RETCODE_OK == data_reader->take_next_sample(&data, &info))
        {
            if (info.valid_data)
            {
                // Do something with the data
                std::cout << "Received new data value for topic "
                          << topic->get_name()
                          << std::endl;
            }
            else
            {
                // If the remote writer is not alive, we exit the reading loop
                std::cout << "Remote writer for topic "
                          << topic->get_name()
                          << " is dead" << std::endl;
                break;
            }
        }
    }
    else
    {
        std::cout << "No data this time" << std::endl;
    }
}
```
