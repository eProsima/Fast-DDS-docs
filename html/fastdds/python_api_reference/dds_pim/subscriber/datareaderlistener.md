<a id="python-api-pim-datareaderlistener"></a>

<a id="datareaderlistener"></a>

# DataReaderListener

### *class* fastdds.DataReaderListener

Class DataReaderListener, it should be used by the end user to implement specific callbacks to certain actions.

#### on_data_available(reader)

Virtual function to be implemented by the user containing the actions to be performed when new Data Messages are received.

* **Parameters:**
  **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – DataReader

#### on_liveliness_changed(reader, status)

Method called when the liveliness status associated to a subscriber changes

* **Parameters:**
  * **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – The DataReader
  * **status** ([`LivelinessChangedStatus`](../core/status/livelinesschangedstatus.md#fastdds.LivelinessChangedStatus)) – The liveliness changed status

#### on_requested_deadline_missed(reader, status)

Virtual method to be called when a topic misses the deadline period

* **Parameters:**
  * **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – DataReader
  * **status** (`RequestedDeadlineMissedStatus`) – The requested deadline missed status

#### on_requested_incompatible_qos(reader, status)

Method called an incompatible QoS was requested.

* **Parameters:**
  * **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – The DataReader
  * **status** (`RequestedIncompatibleQosStatus`) – The requested incompatible QoS status

#### on_sample_lost(reader, status)

Method called when a sample was lost.

* **Parameters:**
  * **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – The DataReader
  * **status** (`SampleLostStatus`) – The sample lost status

#### on_sample_rejected(reader, status)

Method called when a sample was rejected.

* **Parameters:**
  * **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – The DataReader
  * **status** ([`SampleRejectedStatus`](../core/status/samplerejectedstatus.md#fastdds.SampleRejectedStatus)) – The rejected status

#### on_subscription_matched(reader, info)

Virtual method to be called when the subscriber is matched with a new Writer (or unmatched); i.e., when a writer publishing in the same topic is discovered.

* **Parameters:**
  * **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – DataReader
  * **info** ([`SubscriptionMatchedStatus`](../core/status/subscriptionmatchedstatus.md#fastdds.SubscriptionMatchedStatus)) – The subscription matched status

#### *property* thisown

The membership flag
