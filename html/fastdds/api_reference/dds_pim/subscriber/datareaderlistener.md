<a id="api-pim-datareaderlistener"></a>

<a id="datareaderlistener"></a>

# DataReaderListener

### class DataReaderListener

Class [DataReaderListener](#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener), it should be used by the end user to implement specific callbacks to certain actions. 

Subclassed by [eprosima::fastdds::dds::SubscriberListener](subscriberlistener.md#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener)

### Public Functions

### inline DataReaderListener()

Constructor. 

### inline virtual ~DataReaderListener()

Destructor. 

### inline virtual void on_data_available([DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader)

Virtual function to be implemented by the user containing the actions to be performed when new Data Messages are received.

* **Parameters:**
  **reader** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

### inline virtual void on_subscription_matched([DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader, const fastdds::dds::[SubscriptionMatchedStatus](../core/status/subscriptionmatchedstatus.md#_CPPv4N8eprosima7fastdds3dds25SubscriptionMatchedStatusE) &info)

Virtual method to be called when the subscriber is matched with a new Writer (or unmatched); i.e., when a writer publishing in the same topic is discovered.

* **Parameters:**
  * **reader** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
  * **info** – The subscription matched status 

### inline virtual void on_requested_deadline_missed([DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader, const [RequestedDeadlineMissedStatus](../core/status/requesteddeadlinemissedstatus.md#_CPPv4N8eprosima7fastdds3dds29RequestedDeadlineMissedStatusE) &status)

Virtual method to be called when a topic misses the deadline period

* **Parameters:**
  * **reader** – [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
  * **status** – The requested deadline missed status 

### inline virtual void on_liveliness_changed([DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader, const [LivelinessChangedStatus](../core/status/livelinesschangedstatus.md#_CPPv4N8eprosima7fastdds3dds23LivelinessChangedStatusE) &status)

Method called when the liveliness status associated to a subscriber changes. 

* **Parameters:**
  * **reader** – The [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
  * **status** – The liveliness changed status 

### inline virtual void on_sample_rejected([DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader, const [SampleRejectedStatus](../core/status/samplerejectedstatus.md#_CPPv4N8eprosima7fastdds3dds20SampleRejectedStatusE) &status)

Method called when a sample was rejected. 

* **Parameters:**
  * **reader** – The [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
  * **status** – The rejected status 

### inline virtual void on_requested_incompatible_qos([DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader, const [RequestedIncompatibleQosStatus](../core/status/requestedicompatibleqosstatus.md#_CPPv4N8eprosima7fastdds3dds30RequestedIncompatibleQosStatusE) &status)

Method called an incompatible QoS was requested. 

* **Parameters:**
  * **reader** – The [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
  * **status** – The requested incompatible QoS status 

### inline virtual void on_sample_lost([DataReader](datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*reader, const [SampleLostStatus](../core/status/sampleloststatus.md#_CPPv4N8eprosima7fastdds3dds16SampleLostStatusE) &status)

Method called when a sample was lost. 

* **Parameters:**
  * **reader** – The [DataReader](datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)
  * **status** – The sample lost status
