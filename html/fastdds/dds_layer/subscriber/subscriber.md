<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber"></a>

<a id="subscriber"></a>

# Subscriber

A subscription is defined by the association of a [DataReader](dataReader/dataReader.md#dds-layer-subscriber-datareader)
to a [Subscriber](subscriber/subscriber.md#dds-layer-subscriber-subscriber).
To start receiving updates of a publication, the application creates a new
DataReader in a Subscriber.
This DataReader will be bound to the [Topic](../topic/topic/topic.md#dds-layer-topic-topic)
that describes the data type that is going to be received.
The DataReader will then start receiving data value updates from
remote publications that match this Topic.

When the Subscriber receives data, it informs the application that new data is available.
Then, the application can use the DataReader to get the received data.

![image](01-figures/subscriber_class_diagram.svg)

*Subscriber class diagram*

* [Subscriber](subscriber/subscriber.md)
  * [SubscriberQos](subscriber/subscriber.md#subscriberqos)
    * [Default SubscriberQos](subscriber/subscriber.md#default-subscriberqos)
* [SubscriberListener](subscriberListener/subscriberListener.md)
* [Creating a Subscriber](subscriber/createSubscriber.md)
  * [Profile based creation of a Subscriber](subscriber/createSubscriber.md#profile-based-creation-of-a-subscriber)
  * [Deleting a Subscriber](subscriber/createSubscriber.md#deleting-a-subscriber)
* [DataReader](dataReader/dataReader.md)
  * [DataReaderQos](dataReader/dataReader.md#datareaderqos)
    * [Default DataReaderQos](dataReader/dataReader.md#default-datareaderqos)
  * [Type support context](dataReader/dataReader.md#type-support-context)
* [DataReaderListener](dataReaderListener/dataReaderListener.md)
* [Creating a DataReader](dataReader/createDataReader.md)
  * [Profile based creation of a DataReader](dataReader/createDataReader.md#profile-based-creation-of-a-datareader)
  * [Creating a DataWriter with a custom PayloadPool](dataReader/createDataReader.md#creating-a-datawriter-with-a-custom-payloadpool)
  * [Deleting a DataReader](dataReader/createDataReader.md#deleting-a-datareader)
* [SampleInfo](sampleInfo/sampleInfo.md)
  * [sample_state](sampleInfo/sampleInfo.md#sample-state)
  * [view_state](sampleInfo/sampleInfo.md#view-state)
  * [instance_state](sampleInfo/sampleInfo.md#instance-state)
  * [disposed_generation_count](sampleInfo/sampleInfo.md#disposed-generation-count)
  * [no_writers_generation_count](sampleInfo/sampleInfo.md#no-writers-generation-count)
  * [sample_rank](sampleInfo/sampleInfo.md#sample-rank)
  * [generation_rank](sampleInfo/sampleInfo.md#generation-rank)
  * [absolute_generation_rank](sampleInfo/sampleInfo.md#absolute-generation-rank)
  * [source_timestamp](sampleInfo/sampleInfo.md#source-timestamp)
  * [instance_handle](sampleInfo/sampleInfo.md#instance-handle)
  * [publication_handle](sampleInfo/sampleInfo.md#publication-handle)
  * [valid_data](sampleInfo/sampleInfo.md#valid-data)
  * [sample_identity](sampleInfo/sampleInfo.md#sample-identity)
  * [related_sample_identity](sampleInfo/sampleInfo.md#related-sample-identity)
* [Accessing received data](dataReader/readingData.md)
  * [Loaning and Returning Data and SampleInfo Sequences](dataReader/readingData.md#loaning-and-returning-data-and-sampleinfo-sequences)
  * [Processing returned data](dataReader/readingData.md#processing-returned-data)
  * [Accessing data on callbacks](dataReader/readingData.md#accessing-data-on-callbacks)
  * [Accessing data with a waiting thread](dataReader/readingData.md#accessing-data-with-a-waiting-thread)
    * [Wait-sets and DataAvailable status condition](dataReader/readingData.md#wait-sets-and-dataavailable-status-condition)
    * [DataReader non-blocking calls](dataReader/readingData.md#datareader-non-blocking-calls)
