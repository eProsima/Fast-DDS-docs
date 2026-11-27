<a id="python-api-pim-datawriterlistener"></a>

<a id="datawriterlistener"></a>

# DataWriterListener

### *class* fastdds.DataWriterListener

Class DataWriterListener, allows the end user to implement callbacks triggered by certain events.

#### on_liveliness_lost(writer, status)

Method called when the liveliness of a DataWriter is lost

* **Parameters:**
  * **writer** ([`DataWriter`](datawriter.md#fastdds.DataWriter)) – Pointer to the associated DataWriter
  * **status** (`LivelinessLostStatus`) – The liveliness lost status

#### on_offered_deadline_missed(writer, status)

A method called when a deadline is missed

* **Parameters:**
  * **writer** ([`DataWriter`](datawriter.md#fastdds.DataWriter)) – Pointer to the associated DataWriter
  * **status** (`OfferedDeadlineMissedStatus`) – The deadline missed status

#### on_offered_incompatible_qos(writer, status)

A method called when an incompatible QoS is offered

* **Parameters:**
  * **writer** ([`DataWriter`](datawriter.md#fastdds.DataWriter)) – Pointer to the associated DataWriter
  * **status** (`OfferedIncompatibleQosStatus`) – The deadline missed status

#### on_publication_matched(writer, info)

This method is called when the DataWriter is matched (or unmatched) against an endpoint.

* **Parameters:**
  * **writer** ([`DataWriter`](datawriter.md#fastdds.DataWriter)) – Pointer to the associated DataWriter
  * **info** ([`PublicationMatchedStatus`](../core/status/publicationmatchedstatus.md#fastdds.PublicationMatchedStatus)) – Information regarding the matched DataReader

#### on_unacknowledged_sample_removed(writer, instance)

Method called when a sample has been removed unacknowledged

* **Parameters:**
  * **writer** ([`DataWriter`](datawriter.md#fastdds.DataWriter)) – Pointer to the associated DataWriter
  * **instance** (`InstanceHandle_t`) – Handle to the instance the sample was removed from

#### *property* thisown

The membership flag
