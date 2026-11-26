<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher"></a>

<a id="publisher"></a>

# Publisher

A publication is defined by the association of a [DataWriter](dataWriter/dataWriter.md#dds-layer-publisher-datawriter)
to a [Publisher](publisher/publisher.md#dds-layer-publisher-publisher).
To start publishing the values of a data instance, the application creates a new
DataWriter in a Publisher.
This DataWriter will be bound to the [Topic](../topic/topic.md#dds-layer-topic)
that describes the data type that is being transmitted.
Remote subscriptions that match with this
Topic will be able to receive the data value updates from the DataWriter.

![image](01-figures/publisher_class_diagram.svg)

* [Publisher](publisher/publisher.md)
  * [PublisherQos](publisher/publisher.md#publisherqos)
    * [Default PublisherQos](publisher/publisher.md#default-publisherqos)
* [PublisherListener](publisherListener/publisherListener.md)
* [Creating a Publisher](publisher/createPublisher.md)
  * [Profile based creation of a Publisher](publisher/createPublisher.md#profile-based-creation-of-a-publisher)
  * [Deleting a Publisher](publisher/createPublisher.md#deleting-a-publisher)
* [DataWriter](dataWriter/dataWriter.md)
  * [DataWriterQos](dataWriter/dataWriter.md#datawriterqos)
    * [Default DataWriterQos](dataWriter/dataWriter.md#default-datawriterqos)
  * [Type support context](dataWriter/dataWriter.md#type-support-context)
* [DataWriterListener](dataWriterListener/dataWriterListener.md)
  * [on_unacknowledged_sample_removed callback](dataWriterListener/dataWriterListener.md#on-unacknowledged-sample-removed-callback)
* [Creating a DataWriter](dataWriter/createDataWriter.md)
  * [Profile based creation of a DataWriter](dataWriter/createDataWriter.md#profile-based-creation-of-a-datawriter)
  * [Creating a DataWriter with a custom PayloadPool](dataWriter/createDataWriter.md#creating-a-datawriter-with-a-custom-payloadpool)
  * [Deleting a DataWriter](dataWriter/createDataWriter.md#deleting-a-datawriter)
* [Publishing data](dataWriter/publishingData.md)
  * [Blocking of the write operation](dataWriter/publishingData.md#blocking-of-the-write-operation)
  * [Borrowing a data buffer](dataWriter/publishingData.md#borrowing-a-data-buffer)
  * [Prefiltering out DataReaders](dataWriter/publishingData.md#prefiltering-out-datareaders)
