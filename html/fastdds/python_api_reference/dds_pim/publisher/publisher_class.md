<a id="python-api-pim-publisher-class"></a>

<a id="publisher"></a>

# Publisher

### *class* fastdds.Publisher(\*args, \*\*kwargs)

Class Publisher, used to send data to associated subscribers.

#### begin_coherent_changes()

Signals the beginning of a set of coherent cache changes using the Datawriters attached to the publisher

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, an error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### *static* copy_from_topic_qos(writer_qos, topic_qos)

Copies TopicQos into the corresponding DataWriterQos

* **Parameters:**
  * **[****out****]** – writer_qos
  * **[****in****]** – topic_qos
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, an error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### create_datawriter(\*args)

This operation creates a DataWriter. The returned DataWriter will be attached and belongs to the Publisher.

* **Parameters:**
  * **topic** ([`Topic`](../topic/topic_class.md#fastdds.Topic)) – Topic the DataWriter will be listening
  * **qos** – QoS of the DataWriter.
  * **listener** ([`DataWriterListener`](datawriterlistener.md#fastdds.DataWriterListener), optional) – Pointer to the listener (default: nullptr).
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all).
* **Return type:**
  [`DataWriter`](datawriter.md#fastdds.DataWriter)
* **Returns:**
  Pointer to the created DataWriter. nullptr if failed.

#### create_datawriter_with_profile(\*args)

This operation creates a DataWriter. The returned DataWriter will be attached and belongs to the Publisher.

* **Parameters:**
  * **topic** ([`Topic`](../topic/topic_class.md#fastdds.Topic)) – Topic the DataWriter will be listening
  * **profile_name** (*string*) – DataWriter profile name.
  * **listener** ([`DataWriterListener`](datawriterlistener.md#fastdds.DataWriterListener), optional) – Pointer to the listener (default: nullptr).
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all).
* **Return type:**
  [`DataWriter`](datawriter.md#fastdds.DataWriter)
* **Returns:**
  Pointer to the created DataWriter. nullptr if failed.

#### delete_contained_entities()

Deletes all contained DataWriters

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, an error code otherwise

#### delete_datawriter(writer)

This operation deletes a DataWriter that belongs to the Publisher.

The delete_datawriter operation must be called on the same Publisher object used to create the DataWriter.
If delete_datawriter is called on a different Publisher, the operation will have no effect and it will
return false.

The deletion of the DataWriter will automatically unregister all instances.
Depending on the settings of the WRITER_DATA_LIFECYCLE QosPolicy, the deletion of the DataWriter
may also dispose all instances.

* **Parameters:**
  **writer** ([`DataWriter`](datawriter.md#fastdds.DataWriter)) – DataWriter to delete
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if it does not belong to this Publisher, RETCODE_OK if it is correctly deleted and
  RETCODE_ERROR otherwise.

#### enable()

This operation enables the Publisher

* **Return type:**
  int
* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the participant creating this
  Publisher is not enabled.

#### end_coherent_changes()

Signals the end of a set of coherent cache changes

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, an error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_datawriter_qos_from_profile(\*args)

*Overload 1:*

Fills the ‘DataWriterQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – DataWriter profile name.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DataWriterQos’ with the values of the XML profile, and also its corresponding topic name (if specified).

* **Parameters:**
  * **profile_name** (*string*) – DataWriter profile name.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_datawriter_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘DataWriterQos’ with the first DataWriter profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DataWriterQos’ with the first DataWriter profile found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 3:*

Fills the ‘DataWriterQos’ with the DataWriter profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
  * **profile_name** (*string*) – DataWriter profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 4:*

Fills the ‘DataWriterQos’ with the DataWriter profile with `profile_name` to be found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
  * **profile_name** (*string*) – DataWriter profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_datawriters(writers)

Fills the given vector with all the datawriters of this publisher.

* **Parameters:**
  **writers** (*DataWriterVector*) – Vector where the DataWriters are returned
* **Return type:**
  boolean
* **Returns:**
  true

#### get_default_datawriter_qos(\*args)

*Overload 1:*

This operation returns the default value of the DataWriter QoS, that is, the QoS policies which will be used
for newly created DataWriter entities in the case where the QoS policies are defaulted in the
create_datawriter operation.

The values retrieved by get_default_datawriter_qos will match the set of values specified on the last
successful call to set_default_datawriter_qos, or else, if the call was never made, the default values.

* **Return type:**
  [`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)
* **Returns:**
  Current default WriterQos

<br/>

*Overload 2:*

This operation retrieves the default value of the DataWriter QoS, that is, the QoS policies which will be used
for newly created DataWriter entities in the case where the QoS policies are defaulted in the
create_datawriter operation.

The values retrieved by get_default_datawriter_qos will match the set of values specified on the last
successful call to set_default_datawriter_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – Reference to the current default WriterQos.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_default_datawriter_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘DataWriterQos’ with the default DataWriter profile found in the provided XML (if there is).

Notes: This method does not update the default datawriter qos (returned by `get_default_datawriter_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DataWriterQos’ with the default DataWriter profile found in the provided XML (if there is), and also its corresponding topic name (if specified).

Notes: This method does not update the default datawriter qos (returned by `get_default_datawriter_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – ‘DataWriterQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_instance_handle()

Returns the Publisher’s handle.

* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  InstanceHandle of this Publisher.

#### get_listener()

Retrieves the attached PublisherListener.

* **Return type:**
  [`PublisherListener`](publisherlistener.md#fastdds.PublisherListener)
* **Returns:**
  PublisherListener pointer

#### get_participant()

This operation returns the DomainParticipant to which the Publisher belongs.

* **Return type:**
  [`DomainParticipant`](../domain/domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  Pointer to the DomainParticipant

#### get_qos(\*args)

*Overload 1:*

Allows accessing the Publisher Qos.

* **Return type:**
  [`PublisherQos`](publisherqos.md#fastdds.PublisherQos)
* **Returns:**
  PublisherQos reference

<br/>

*Overload 2:*

Retrieves the Publisher Qos.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### has_datawriters()

This operation checks if the publisher has DataWriters

* **Return type:**
  boolean
* **Returns:**
  true if the publisher has one or several DataWriters, false otherwise

#### lookup_datawriter(topic_name)

This operation retrieves a previously created DataWriter belonging to the Publisher that is attached to a
Topic with a matching topic_name. If no such DataWriter exists, the operation will return nullptr.

If multiple DataWriter attached to the Publisher satisfy this condition, then the operation will return
one of them. It is not specified which one.

* **Parameters:**
  **topic_name** (*string*) – Name of the Topic
* **Return type:**
  [`DataWriter`](datawriter.md#fastdds.DataWriter)
* **Returns:**
  Pointer to a previously created DataWriter associated to a Topic with the requested topic_name

#### resume_publications()

Indicates to FastDDS that the modifications to the DataWriters are complete.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, an error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### set_default_datawriter_qos(qos)

This operation sets a default value of the DataWriter QoS policies which will be used for newly created
DataWriter entities in the case where the QoS policies are defaulted in the create_datawriter operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation
will have no effect and return false.

The special value DATAWRITER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS
should be reset back to the initial values the factory would use, that is the values that would be used
if the set_default_datawriter_qos operation had never been called.

* **Parameters:**
  **qos** ([`DataWriterQos`](datawriterqos.md#fastdds.DataWriterQos)) – DataWriterQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly.

#### set_listener(\*args)

*Overload 1:*

Modifies the PublisherListener, sets the mask to StatusMask::all()

* **Parameters:**
  **listener** ([`PublisherListener`](publisherlistener.md#fastdds.PublisherListener)) – new value for the PublisherListener
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

<br/>

*Overload 2:*

Modifies the PublisherListener.

* **Parameters:**
  * **listener** ([`PublisherListener`](publisherlistener.md#fastdds.PublisherListener)) – new value for the PublisherListener
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask)) – StatusMask that holds statuses the listener responds to
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### set_qos(qos)

Allows modifying the Publisher Qos.
The given Qos must be supported by the PublisherQos.

* **Parameters:**
  **qos** ([`PublisherQos`](publisherqos.md#fastdds.PublisherQos)) – PublisherQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not
  self consistent and RETCODE_OK if the qos is changed correctly.

#### suspend_publications()

Indicates to FastDDS that the contained DataWriters are about to be modified

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, an error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### *property* thisown

The membership flag

#### wait_for_acknowledgments(max_wait)

This operation blocks the calling thread until either all data written by the reliable DataWriter entities
is acknowledged by all matched reliable DataReader entities, or else the duration specified by the max_wait
parameter elapses, whichever happens first. A return value of true indicates that all the samples written
have been acknowledged by all reliable matched data readers; a return value of false indicates that max_wait
elapsed before all the data was acknowledged.

* **Parameters:**
  **max_wait** (`Duration_t`) – Maximum blocking time for this operation
* **Return type:**
  int
* **Returns:**
  RETCODE_TIMEOUT if the function takes more than the maximum blocking time established, RETCODE_OK if the
  Publisher receives the acknowledgments and RETCODE_ERROR otherwise.
