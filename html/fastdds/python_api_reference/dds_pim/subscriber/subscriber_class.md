<a id="python-api-pim-subscriber-class"></a>

<a id="subscriber"></a>

# Subscriber

### *class* fastdds.Subscriber(\*args, \*\*kwargs)

Class Subscriber, contains the public API that allows the user to control the reception of messages.
This class should not be instantiated directly.
DomainRTPSParticipant class should be used to correctly create this element.

#### begin_access()

Indicates that the application is about to access the data samples in any of the DataReader objects
attached to the Subscriber.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### *static* copy_from_topic_qos(reader_qos, topic_qos)

Copies TopicQos into the corresponding DataReaderQos

:param [in:, out] reader_qos
:param [in]: topic_qos
:rtype: int
:return: RETCODE_OK if successful, an error code otherwise

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### create_datareader(\*args)

This operation creates a DataReader. The returned DataReader will be attached and belong to the Subscriber.

* **Parameters:**
  * **topic** ([`TopicDescription`](../topic/topicdescription.md#fastdds.TopicDescription)) – Topic the DataReader will be listening.
  * **reader_qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – QoS of the DataReader.
  * **listener** ([`DataReaderListener`](datareaderlistener.md#fastdds.DataReaderListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all).
* **Return type:**
  [`DataReader`](datareader.md#fastdds.DataReader)
* **Returns:**
  Pointer to the created DataReader. nullptr if failed.

#### create_datareader_with_profile(\*args)

This operation creates a DataReader. The returned DataReader will be attached and belongs to the Subscriber.

* **Parameters:**
  * **topic** ([`TopicDescription`](../topic/topicdescription.md#fastdds.TopicDescription)) – Topic the DataReader will be listening.
  * **profile_name** (*string*) – DataReader profile name.
  * **listener** ([`DataReaderListener`](datareaderlistener.md#fastdds.DataReaderListener), optional) – Pointer to the listener (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all).
* **Return type:**
  [`DataReader`](datareader.md#fastdds.DataReader)
* **Returns:**
  Pointer to the created DataReader. nullptr if failed.

#### delete_contained_entities()

Deletes all contained DataReaders. If the DataReaders have any QueryCondition or ReadCondition, they are
deleted before the DataReader itself.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if successful, an error code otherwise

#### delete_datareader(reader)

This operation deletes a DataReader that belongs to the Subscriber.

The delete_datareader operation must be called on the same Subscriber object used to create the DataReader.
If delete_datareader is called on a different Subscriber, the operation will have no effect and it will
return an error.

* **Parameters:**
  **reader** ([`DataReader`](datareader.md#fastdds.DataReader)) – DataReader to delete
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the datareader does not belong to this subscriber, RETCODE_OK if it is correctly
  deleted and RETCODE_ERROR otherwise.

#### enable()

This operation enables the Subscriber

* **Return type:**
  int
* **Returns:**
  RETCODE_OK is successfully enabled. RETCODE_PRECONDITION_NOT_MET if the participant creating this
  Subscriber is not enabled.

#### end_access()

Indicates that the application has finished accessing the data samples in DataReader objects managed by
the Subscriber.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_datareader_qos_from_profile(\*args)

*Overload 1:*

Fills the ‘DataReaderQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – DataReader profile name.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DataReaderQos’ with the values of the XML profile, and also its corresponding topic name (if specified).

* **Parameters:**
  * **profile_name** (*string*) – DataReader profile name.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ if the profile exists. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_datareader_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘DataReaderQos’ with the first DataReader profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DataReaderQos’ with the first DataReader profile found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 3:*

Fills the ‘DataReaderQos’ with the DataReader profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
  * **profile_name** (*string*) – DataReader profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 4:*

Fills the ‘DataReaderQos’ with the DataReader profile with `profile_name` to be found in the provided XML, and also its corresponding topic name (if specified).

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
  * **profile_name** (*string*) – DataReader profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_datareaders(\*args)

*Overload 1:*

This operation allows the application to access the DataReader objects.

* **Parameters:**
  **readers** (*DataReaderVector*) – Vector of DataReader where the list of existing readers is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

<br/>

*Overload 2:*

This operation allows the application to access the DataReader objects that contain samples with the
specified sample_states, view_states, and instance_states.

* **Parameters:**
  * **[****out****]** – readers Vector of DataReader where the list of existing readers is returned
  * **sample_states** (*std::vector< eprosima::fastdds::dds::SampleStateKind* *,**std::allocator< eprosima::fastdds::dds::SampleStateKind > >*) – Vector of SampleStateKind
  * **view_states** (*std::vector< eprosima::fastdds::dds::ViewStateKind* *,**std::allocator< eprosima::fastdds::dds::ViewStateKind > >*) – Vector of ViewStateKind
  * **instance_states** (*std::vector< eprosima::fastdds::dds::InstanceStateKind* *,**std::allocator< eprosima::fastdds::dds::InstanceStateKind > >*) – Vector of InstanceStateKind
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

Warning: Not supported yet. Currently returns RETCODE_UNSUPPORTED

#### get_default_datareader_qos(\*args)

*Overload 1:*

This operation returns the default value of the DataReader QoS, that is, the QoS policies which will be
used for newly created DataReader entities in the case where the QoS policies are defaulted in the
create_datareader operation.

The values retrieved get_default_datareader_qos will match the set of values specified on the last successful
call to get_default_datareader_qos, or else, if the call was never made, the default values.

* **Return type:**
  [`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)
* **Returns:**
  Current default DataReaderQos.

<br/>

*Overload 2:*

This operation returns the default value of the DataReader QoS, that is, the QoS policies which will be
used for newly created DataReader entities in the case where the QoS policies are defaulted in the
create_datareader operation.

The values retrieved get_default_datareader_qos will match the set of values specified on the last successful
call to get_default_datareader_qos, or else, if the call was never made, the default values.

* **Return type:**
  [`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)
* **Returns:**
  Current default DataReaderQos.

<br/>

*Overload 3:*

This operation retrieves the default value of the DataReader QoS, that is, the QoS policies which will be
used for newly created DataReader entities in the case where the QoS policies are defaulted in the
create_datareader operation.

The values retrieved get_default_datareader_qos will match the set of values specified on the last successful
call to get_default_datareader_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – DataReaderQos where the default_qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_default_datareader_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘DataReaderQos’ with the default DataReader profile found in the provided XML (if there is).

Notes: This method does not update the default datareader qos (returned by `get_default_datareader_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DataReaderQos’ with the default DataReader profile found in the provided XML (if there is), and also its corresponding topic name (if specified).

Notes: This method does not update the default datareader qos (returned by `get_default_datareader_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – ‘DataReaderQos’ object where the qos is returned.
  * **topic_name** (*string*) – String where the name of the topic associated to this profile is returned (if specified).
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_instance_handle()

Returns the Subscriber’s handle.

* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  InstanceHandle of this Subscriber.

#### get_listener()

Retrieves the attached SubscriberListener.

* **Return type:**
  [`SubscriberListener`](subscriberlistener.md#fastdds.SubscriberListener)
* **Returns:**
  Pointer to the SubscriberListener

#### get_participant()

This operation returns the DomainParticipant to which the Subscriber belongs.

* **Return type:**
  [`DomainParticipant`](../domain/domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant Pointer

#### get_qos(\*args)

*Overload 1:*

Allows accessing the Subscriber Qos.

* **Return type:**
  [`SubscriberQos`](subscriberqos.md#fastdds.SubscriberQos)
* **Returns:**
  SubscriberQos reference

<br/>

*Overload 2:*

Retrieves the Subscriber Qos.

* **Parameters:**
  **qos** ([`SubscriberQos`](subscriberqos.md#fastdds.SubscriberQos)) – SubscriberQos where the qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### has_datareaders()

This operation checks if the subscriber has DataReaders

* **Return type:**
  boolean
* **Returns:**
  true if the subscriber has one or several DataReaders, false in other case

#### lookup_datareader(topic_name)

This operation retrieves a previously-created DataReader belonging to the Subscriber that is attached to a
Topic with a matching topic_name. If no such DataReader exists, the operation will return nullptr.

If multiple DataReaders attached to the Subscriber satisfy this condition, then the operation will return
one of them. It is not specified which one.

* **Parameters:**
  **topic_name** (*string*) – Name of the topic associated to the DataReader
* **Return type:**
  [`DataReader`](datareader.md#fastdds.DataReader)
* **Returns:**
  Pointer to a previously created DataReader created on a Topic with that topic_name

#### notify_datareaders()

This operation invokes the operation on_data_available on the DataReaderListener objects attached to
contained DataReader entities.

This operation is typically invoked from the on_data_on_readers operation in the SubscriberListener.
That way the SubscriberListener can delegate to the DataReaderListener objects the handling of the data.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### set_default_datareader_qos(qos)

This operation sets a default value of the DataReader QoS policies which will be used for newly created
DataReader entities in the case where the QoS policies are defaulted in the create_datareader operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation
will have no effect and return false.

The special value DATAREADER_QOS_DEFAULT may be passed to this operation to indicate that the default QoS
should be reset back to the initial values the factory would use, that is the values that would be used
if the set_default_datareader_qos operation had never been called.

* **Parameters:**
  **qos** ([`DataReaderQos`](datareaderqos.md#fastdds.DataReaderQos)) – new value for DataReaderQos to set as default
* **Return type:**
  int
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly.

#### set_listener(\*args)

*Overload 1:*

Modifies the SubscriberListener, sets the mask to StatusMask::all()

* **Parameters:**
  **listener** ([`SubscriberListener`](subscriberlistener.md#fastdds.SubscriberListener)) – new value for SubscriberListener
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

<br/>

*Overload 2:*

Modifies the SubscriberListener.

* **Parameters:**
  * **listener** ([`SubscriberListener`](subscriberlistener.md#fastdds.SubscriberListener)) – new value for the SubscriberListener
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask)) – StatusMask that holds statuses the listener responds to.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### set_qos(qos)

Allows modifying the Subscriber Qos.
The given Qos must be supported by the SubscriberQos.

* **Parameters:**
  **qos** ([`SubscriberQos`](subscriberqos.md#fastdds.SubscriberQos)) – new value for SubscriberQos
* **Return type:**
  int
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not
  self consistent and RETCODE_OK if the qos is changed correctly.

#### *property* thisown

The membership flag
