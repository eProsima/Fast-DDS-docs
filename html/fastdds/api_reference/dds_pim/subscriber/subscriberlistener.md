<a id="api-pim-subscriberlistener"></a>

<a id="subscriberlistener"></a>

# SubscriberListener

### class SubscriberListener : public eprosima::fastdds::dds::[DataReaderListener](datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE)

Class [SubscriberListener](#classeprosima_1_1fastdds_1_1dds_1_1SubscriberListener), it should be used by the end user to implement specific callbacks to certain actions. It also inherits all [DataReaderListener](datareaderlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DataReaderListener) callbacks. 

Subclassed by [eprosima::fastdds::dds::DomainParticipantListener](../domain/domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener)

### Public Functions

### inline SubscriberListener()

Constructor. 

### inline virtual ~SubscriberListener()

Destructor. 

### inline virtual void on_data_on_readers([Subscriber](subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE) \*sub)

Virtual function to be implemented by the user containing the actions to be performed when a new Data Message is available on any reader.

* **Parameters:**
  **sub** – [Subscriber](subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber)
