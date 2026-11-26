<a id="python-api-pim-domainparticipantfactory"></a>

<a id="domainparticipantfactory"></a>

# DomainParticipantFactory

### *class* fastdds.DomainParticipantFactory(\*args, \*\*kwargs)

Class DomainParticipantFactory

#### check_xml_static_discovery(xml_file)

Check the validity of the provided static discovery XML file

* **Parameters:**
  **xml_file** (*string*) – xml file path
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the validation is successful, RETCODE_ERROR otherwise.

#### create_participant(\*args)

*Overload 1:*

Create a Participant.

* **Parameters:**
  * **domain_id** (*int*) – Domain Id.
  * **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – DomainParticipantQos Reference.
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener), optional) – DomainParticipantListener Pointer (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

<br/>

*Overload 2:*

Create a Participant.

* **Parameters:**
  * **extended_qos** (`DomainParticipantExtendedQos`) – DomainParticipantExtendedQos Reference.
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener), optional) – DomainParticipantListener Pointer (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

<br/>

*Overload 3:*

Create a Participant.

* **Parameters:**
  * **extended_qos** (`DomainParticipantExtendedQos`) – DomainParticipantExtendedQos Reference.
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener), optional) – DomainParticipantListener Pointer (default: nullptr)
  * **mask** – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

<br/>

*Overload 4:*

Create a Participant.

* **Parameters:**
  * **extended_qos** (`DomainParticipantExtendedQos`) – DomainParticipantExtendedQos Reference.
  * **listener** – DomainParticipantListener Pointer (default: nullptr)
  * **mask** – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

#### create_participant_with_default_profile(\*args)

*Overload 1:*

Create a Participant with default domain id and qos.

* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

<br/>

*Overload 2:*

Create a Participant with default domain id and qos.

* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)
* **Parameters:**
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener)) – DomainParticipantListener Pointer
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask)) – StatusMask Reference

#### create_participant_with_profile(\*args)

*Overload 1:*

Create a Participant.

* **Parameters:**
  * **domain_id** (*int*) – Domain Id.
  * **profile_name** (*string*) – Participant profile name.
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener), optional) – DomainParticipantListener Pointer (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

<br/>

*Overload 2:*

Create a Participant.

* **Parameters:**
  * **profile_name** (*string*) – Participant profile name.
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener), optional) – DomainParticipantListener Pointer (default: nullptr)
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

<br/>

*Overload 3:*

Create a Participant.

* **Parameters:**
  * **profile_name** (*string*) – Participant profile name.
  * **listener** ([`DomainParticipantListener`](domainparticipantlistener.md#fastdds.DomainParticipantListener), optional) – DomainParticipantListener Pointer (default: nullptr)
  * **mask** – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

<br/>

*Overload 4:*

Create a Participant.

* **Parameters:**
  * **profile_name** (*string*) – Participant profile name.
  * **listener** – DomainParticipantListener Pointer (default: nullptr)
  * **mask** – StatusMask Reference (default: all)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer. (nullptr if not created.)

#### delete_participant(part)

Remove a Participant and all associated publishers and subscribers.

* **Parameters:**
  **part** ([`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)) – Pointer to the participant.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the participant has active entities, RETCODE_OK if the participant is correctly
  deleted and RETCODE_ERROR otherwise.

#### get_default_participant_extended_qos_from_xml(xml, extended_qos)

Fills the ‘DomainParticipantExtendedQos’ with the default DomainParticipant profile found in the provided XML (if there is).

Notes: This method does not update the default participant extended qos.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `extended_qos` structure.
  * **extended_qos** (`DomainParticipantExtendedQos`) – ‘DomainParticipantExtendedQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_default_participant_qos(\*args)

*Overload 1:*

This operation retrieves the default value of the DomainParticipant QoS, that is, the QoS policies which will
be used for newly created DomainParticipant entities in the case where the QoS policies are defaulted in the
create_participant operation.
The values retrieved get_default_participant_qos will match the set of values specified on the last successful call
to set_default_participant_qos, or else, if the call was never made, the default values.

* **Parameters:**
  **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – DomainParticipantQos where the qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

<br/>

*Overload 2:*

This operation retrieves the default value of the DomainParticipant QoS, that is, the QoS policies which will
be used for newly created DomainParticipant entities in the case where the QoS policies are defaulted in the
create_participant operation.
The values retrieved get_default_participant_qos will match the set of values specified on the last successful call
to set_default_participant_qos, or else, if the call was never made, the default values.

* **Return type:**
  [`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)
* **Returns:**
  A reference to the default DomainParticipantQos

#### get_default_participant_qos_from_xml(xml, qos)

Fills the ‘DomainParticipantQos’ with the default DomainParticipant profile found in the provided XML (if there is).

Notes: This method does not update the default participant qos (returned by `get_default_participant_qos)`.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – ‘DomainParticipantQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### *static* get_instance()

Returns the DomainParticipantFactory singleton instance.

* **Return type:**
  [`DomainParticipantFactory`](#fastdds.DomainParticipantFactory)
* **Returns:**
  A raw pointer to the DomainParticipantFactory singleton instance.

#### get_library_settings(library_settings)

This operation returns the value of the DomainParticipant library settings.

* **Parameters:**
  **library_settings** (`LibrarySettings`) – LibrarySettings reference where the settings are returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_participant_extended_qos_from_default_profile(extended_qos)

Fills the ‘DomainParticipantExtendedQos’ with the values of the default XML profile.

* **Parameters:**
  **extended_qos** (`DomainParticipantExtendedQos`) – ‘DomainParticipantExtendedQos’ object where the domain and qos are returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_participant_extended_qos_from_profile(profile_name, extended_qos)

Fills the ‘DomainParticipantExtendedQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – DomainParticipant profile name.
  * **extended_qos** (`DomainParticipantExtendedQos`) – DomainParticipantExtendedQos object where the domain and qos are returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise.

#### get_participant_extended_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘DomainParticipantExtendedQos’ with the first DomainParticipant profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `extended_qos` structure.
  * **extended_qos** (`DomainParticipantExtendedQos`) – ‘DomainParticipantExtendedQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DomainParticipantExtendedQos’ with the DomainParticipant profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `extended_qos` structure.
  * **extended_qos** (`DomainParticipantExtendedQos`) – ‘DomainParticipantExtendedQos’ object where the qos is returned.
  * **profile_name** (*string*) – DomainParticipant profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_participant_qos_from_profile(profile_name, qos)

Fills the ‘DomainParticipantQos’ with the values of the XML profile.

* **Parameters:**
  * **profile_name** (*string*) – DomainParticipant profile name.
  * **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – ‘DomainParticipantQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise.

#### get_participant_qos_from_xml(\*args)

*Overload 1:*

Fills the ‘DomainParticipantQos’ with the first DomainParticipant profile found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – ‘DomainParticipantQos’ object where the qos is returned.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

<br/>

*Overload 2:*

Fills the ‘DomainParticipantQos’ with the DomainParticipant profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** (*string*) – Raw XML string containing the profile to be used to fill the `qos` structure.
  * **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – ‘DomainParticipantQos’ object where the qos is returned.
  * **profile_name** (*string*) – DomainParticipant profile name.
* **Return type:**
  int
* **Returns:**
  ‘RETCODE_OK’ on success. ‘RETCODE_BAD_PARAMETER’ otherwise.

#### get_qos(qos)

This operation returns the value of the DomainParticipantFactory QoS policies.

* **Parameters:**
  **qos** ([`DomainParticipantFactoryQos`](domainparticipantfactoryqos.md#fastdds.DomainParticipantFactoryQos)) – DomaParticipantFactoryQos reference where the qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### *static* get_shared_instance()

Returns the DomainParticipantFactory singleton instance.

* **Return type:**
  std::shared_ptr< eprosima::fastdds::dds::DomainParticipantFactory >
* **Returns:**
  A shared pointer to the DomainParticipantFactory singleton instance.

#### load_XML_profiles_file(xml_profile_file)

Load profiles from XML file.

* **Parameters:**
  **xml_profile_file** (*string*) – XML profile file.
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if it is correctly loaded, RETCODE_ERROR otherwise.

#### load_XML_profiles_string(data, length)

Load profiles from XML string.

* **Parameters:**
  * **data** (*string*) – buffer containing xml data.
  * **length** (*std::size_t*) – length of data
* **Return type:**
  int
* **Returns:**
  RETCODE_OK if it is correctly loaded, RETCODE_ERROR otherwise.

#### load_profiles()

Load profiles from default XML file.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### lookup_participant(domain_id)

This operation retrieves a previously created DomainParticipant belonging to specified domain_id.
If no such DomainParticipant exists, the operation will return ‘nullptr’.
If multiple DomainParticipant entities belonging to that domain_id exist,
then the operation will return one of them. It is not specified which one.

* **Parameters:**
  **domain_id** (*int*)
* **Return type:**
  [`DomainParticipant`](domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  previously created DomainParticipant within the specified domain

#### lookup_participants(domain_id)

Returns all participants that belongs to the specified domain_id.

* **Parameters:**
  **domain_id** (*int*)
* **Return type:**
  DomainParticipantVector
* **Returns:**
  previously created DomainParticipants within the specified domain

#### set_default_participant_qos(qos)

This operation sets a default value of the DomainParticipant QoS policies which will be used for
newly created DomainParticipant entities in the case where the QoS policies are defaulted in the
create_participant operation.

This operation will check that the resulting policies are self consistent; if they are not, the operation
will have no effect and return INCONSISTENT_POLICY.

The special value PARTICIPANT_QOS_DEFAULT may be passed to this operation to indicate that the default
QoS should be reset back to the initial values the factory would use, that is the values that would be
used if the set_default_participant_qos operation had never been called.

* **Parameters:**
  **qos** ([`DomainParticipantQos`](domainparticipantqos.md#fastdds.DomainParticipantQos)) – DomainParticipantQos to be set
* **Return type:**
  int
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly.

#### set_library_settings(library_settings)

This operation sets the library settings.

Library settings must be set before enabling the DomainParticipants.
Otherwise, failure of the setting operation is expected.

* **Parameters:**
  **library_settings** (`LibrarySettings`) – LibrarySettings to be set.
* **Return type:**
  int
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if any DomainParticipant is already enabled.
  RETCODE_OK otherwise.

#### set_qos(qos)

This operation sets the value of the DomainParticipantFactory QoS policies. These policies
control the behavior of the object a factory for entities.

Note that despite having QoS, the DomainParticipantFactory is not an Entity.

This operation will check that the resulting policies are self consistent; if they are not,
the operation will have no effect and return INCONSISTENT_POLICY.

* **Parameters:**
  **qos** ([`DomainParticipantFactoryQos`](domainparticipantfactoryqos.md#fastdds.DomainParticipantFactoryQos)) – DomainParticipantFactoryQos to be set.
* **Return type:**
  int
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not
  self consistent and RETCODE_OK if the qos is changed correctly.

#### *property* thisown

The membership flag

#### type_object_registry()

Return the TypeObjectRegistry member to access the public API.

* **Return type:**
  eprosima::fastdds::dds::xtypes::ITypeObjectRegistry
* **Returns:**
  const xtypes::TypeObjectRegistry reference.
