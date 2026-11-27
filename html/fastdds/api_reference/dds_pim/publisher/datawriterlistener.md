<a id="api-pim-datawriterlistener"></a>

<a id="datawriterlistener"></a>

# DataWriterListener

### class DataWriterListener

Class [DataWriterListener](#classeprosima_1_1fastdds_1_1dds_1_1DataWriterListener), allows the end user to implement callbacks triggered by certain events. 

Subclassed by [eprosima::fastdds::dds::PublisherListener](publisherlistener.md#classeprosima_1_1fastdds_1_1dds_1_1PublisherListener)

### Public Functions

### inline DataWriterListener()

Constructor. 

### inline virtual ~DataWriterListener()

Destructor. 

### inline virtual void on_publication_matched([DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*writer, const [PublicationMatchedStatus](../core/status/publicationmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds24PublicationMatchedStatusE) &info)

This method is called when the [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) is matched (or unmatched) against an endpoint.

* **Parameters:**
  * **writer** – Pointer to the associated [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)
  * **info** – Information regarding the matched [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

### inline virtual void on_offered_deadline_missed([DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*writer, const [OfferedDeadlineMissedStatus](../core/status/offereddeadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds27OfferedDeadlineMissedStatusE) &status)

A method called when a deadline is missed

* **Parameters:**
  * **writer** – Pointer to the associated [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)
  * **status** – The deadline missed status 

### inline virtual void on_offered_incompatible_qos([DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*writer, const [OfferedIncompatibleQosStatus](../core/status/offeredicompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds28OfferedIncompatibleQosStatusE) &status)

A method called when an incompatible QoS is offered

* **Parameters:**
  * **writer** – Pointer to the associated [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)
  * **status** – The deadline missed status 

### inline virtual void on_liveliness_lost([DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*writer, const [LivelinessLostStatus](../core/status/rtpslivelinessloststatus.md#_CPPv4N8eprosima7fastdds3dds20LivelinessLostStatusE) &status)

Method called when the liveliness of a [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) is lost. 

* **Parameters:**
  * **writer** – Pointer to the associated [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)
  * **status** – The liveliness lost status 

### inline virtual void on_unacknowledged_sample_removed([DataWriter](datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*writer, const InstanceHandle_t &instance)

Method called when a sample has been removed unacknowledged. 

* **Parameters:**
  * **writer** – Pointer to the associated [DataWriter](datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)
  * **instance** – Handle to the instance the sample was removed from
