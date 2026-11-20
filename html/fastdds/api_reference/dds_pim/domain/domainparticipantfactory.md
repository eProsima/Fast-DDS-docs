<a id="api-pim-domainparticipantfactory"></a>

<a id="domainparticipantfactory"></a>

# DomainParticipantFactory

### class DomainParticipantFactory

Class [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory)

### Public Functions

### [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*create_participant(DomainId_t domain_id, const [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos, [DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a Participant.

* **Parameters:**
  * **domain_id** – Domain Id. 
  * **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) Reference. 
  * **listener** – [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener) Pointer (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) Reference (default: all) 
* **Returns:**
  [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer. (nullptr if not created.) 

### [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*create_participant(const [DomainParticipantExtendedQos](domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) &extended_qos, [DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a Participant.

* **Parameters:**
  * **extended_qos** – [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) Reference. 
  * **listener** – [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener) Pointer (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) Reference (default: all) 
* **Returns:**
  [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer. (nullptr if not created.) 

### [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*create_participant_with_default_profile()

Create a Participant with default domain id and qos.

* **Returns:**
  [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer. (nullptr if not created.) 

### [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*create_participant_with_default_profile([DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask)

Create a Participant with default domain id and qos.

* **Parameters:**
  * **listener** – [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener) Pointer 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) Reference 
* **Returns:**
  [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer. (nullptr if not created.) 

### [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*create_participant_with_profile(DomainId_t domain_id, const std::string &profile_name, [DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a Participant.

* **Parameters:**
  * **domain_id** – Domain Id. 
  * **profile_name** – Participant profile name. 
  * **listener** – [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener) Pointer (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) Reference (default: all) 
* **Returns:**
  [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer. (nullptr if not created.) 

### [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*create_participant_with_profile(const std::string &profile_name, [DomainParticipantListener](domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListenerE) \*listener = nullptr, const [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](../core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Create a Participant.

* **Parameters:**
  * **profile_name** – Participant profile name. 
  * **listener** – [DomainParticipantListener](domainparticipantlistener.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantListener) Pointer (default: nullptr) 
  * **mask** – [StatusMask](../core/status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) Reference (default: all) 
* **Returns:**
  [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer. (nullptr if not created.) 

### [DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*lookup_participant(DomainId_t domain_id) const

This operation retrieves a previously created [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) belonging to specified domain_id. If no such [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) exists, the operation will return ‘nullptr’. If multiple [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) entities belonging to that domain_id exist, then the operation will return one of them. It is not specified which one.

* **Parameters:**
  **domain_id** – 
* **Returns:**
  previously created [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) within the specified domain 

### std::vector<[DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE)\*> lookup_participants(DomainId_t domain_id) const

Returns all participants that belongs to the specified domain_id.

* **Parameters:**
  **domain_id** – 
* **Returns:**
  previously created DomainParticipants within the specified domain 

### ReturnCode_t get_default_participant_qos([DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos) const

This operation retrieves the default value of the [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) QoS, that is, the QoS policies which will be used for newly created [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) entities in the case where the QoS policies are defaulted in the create_participant operation. The values retrieved get_default_participant_qos will match the set of values specified on the last successful call to set_default_participant_qos, or else, if the call was never made, the default values. 

* **Parameters:**
  **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) where the qos is returned 
* **Returns:**
  RETCODE_OK 

### const [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &get_default_participant_qos() const

This operation retrieves the default value of the [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) QoS, that is, the QoS policies which will be used for newly created [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) entities in the case where the QoS policies are defaulted in the create_participant operation. The values retrieved get_default_participant_qos will match the set of values specified on the last successful call to set_default_participant_qos, or else, if the call was never made, the default values. 

* **Returns:**
  A reference to the default [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos)

### ReturnCode_t set_default_participant_qos(const [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos)

This operation sets a default value of the [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) QoS policies which will be used for newly created [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) entities in the case where the QoS policies are defaulted in the create_participant operation. 

This operation will check that the resulting policies are self consistent; if they are not, the operation will have no effect and return INCONSISTENT_POLICY.

The special value PARTICIPANT_QOS_DEFAULT may be passed to this operation to indicate that the default QoS should be reset back to the initial values the factory would use, that is the values that would be used if the set_default_participant_qos operation had never been called.

* **Parameters:**
  **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) to be set 
* **Returns:**
  RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### ReturnCode_t get_participant_qos_from_profile(const std::string &profile_name, [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos) const

Fills the [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile name. 
  * **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_participant_qos_from_xml(const std::string &xml, [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos) const

Fills the [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) with the first [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_participant_qos_from_xml(const std::string &xml, [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos, const std::string &profile_name) const

Fills the [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) with the [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) object where the qos is returned. 
  * **profile_name** – [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_participant_qos_from_xml(const std::string &xml, [DomainParticipantQos](domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos) const

Fills the [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) with the default [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile found in the provided XML (if there is).

#### NOTE
This method does not update the default participant qos (returned by `get_default_participant_qos`).

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `qos` structure. 
  * **qos** – [DomainParticipantQos](domainparticipantqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_participant_extended_qos_from_profile(const std::string &profile_name, [DomainParticipantExtendedQos](domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) &extended_qos) const

Fills the [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) with the values of the XML profile.

* **Parameters:**
  * **profile_name** – [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile name. 
  * **extended_qos** – [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) object where the domain and qos are returned. 
* **Returns:**
  RETCODE_OK if the profile exists. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_participant_extended_qos_from_xml(const std::string &xml, [DomainParticipantExtendedQos](domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) &extended_qos) const

Fills the [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) with the first [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `extended_qos` structure. 
  * **extended_qos** – [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_participant_extended_qos_from_xml(const std::string &xml, [DomainParticipantExtendedQos](domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) &extended_qos, const std::string &profile_name) const

Fills the [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) with the [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile with `profile_name` to be found in the provided XML.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `extended_qos` structure. 
  * **extended_qos** – [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) object where the qos is returned. 
  * **profile_name** – [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile name. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_default_participant_extended_qos_from_xml(const std::string &xml, [DomainParticipantExtendedQos](domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) &extended_qos) const

Fills the [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) with the default [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) profile found in the provided XML (if there is).

#### NOTE
This method does not update the default participant extended qos.

* **Parameters:**
  * **xml** – Raw XML string containing the profile to be used to fill the `extended_qos` structure. 
  * **extended_qos** – [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) object where the qos is returned. 
* **Returns:**
  RETCODE_OK on success. RETCODE_BAD_PARAMETER otherwise. 

### ReturnCode_t get_participant_extended_qos_from_default_profile([DomainParticipantExtendedQos](domainparticipantextendedqos.md#_CPPv4N8eprosima7fastdds3dds28DomainParticipantExtendedQosE) &extended_qos) const

Fills the [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) with the values of the default XML profile.

* **Parameters:**
  **extended_qos** – [DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos) object where the domain and qos are returned. 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t delete_participant([DomainParticipant](domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*part)

Remove a Participant and all associated publishers and subscribers.

* **Parameters:**
  **part** – Pointer to the participant. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the participant has active entities, RETCODE_OK if the participant is correctly deleted and RETCODE_ERROR otherwise. 

### ReturnCode_t load_profiles()

Load profiles from default XML file.

* **Returns:**
  RETCODE_OK 

### ReturnCode_t load_XML_profiles_file(const std::string &xml_profile_file)

Load profiles from XML file.

* **Parameters:**
  **xml_profile_file** – XML profile file. 
* **Returns:**
  RETCODE_OK if it is correctly loaded, RETCODE_ERROR otherwise. 

### ReturnCode_t load_XML_profiles_string(const char \*data, std::size_t length)

Load profiles from XML string.

* **Parameters:**
  * **data** – buffer containing xml data. 
  * **length** – length of data 
* **Returns:**
  RETCODE_OK if it is correctly loaded, RETCODE_ERROR otherwise. 

### ReturnCode_t check_xml_static_discovery(std::string &xml_file)

Check the validity of the provided static discovery XML file

* **Parameters:**
  **xml_file** – xml file path 
* **Returns:**
  RETCODE_OK if the validation is successful, RETCODE_ERROR otherwise. 

### ReturnCode_t get_qos([DomainParticipantFactoryQos](domainparticipantfactoryqos.md#_CPPv4N8eprosima7fastdds3dds27DomainParticipantFactoryQosE) &qos) const

This operation returns the value of the [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory) QoS policies.

* **Parameters:**
  **qos** – DomaParticipantFactoryQos reference where the qos is returned 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_qos(const [DomainParticipantFactoryQos](domainparticipantfactoryqos.md#_CPPv4N8eprosima7fastdds3dds27DomainParticipantFactoryQosE) &qos)

This operation sets the value of the [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory) QoS policies. These policies control the behavior of the object a factory for entities.

Note that despite having QoS, the [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory) is not an [Entity](../core/entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity).

This operation will check that the resulting policies are self consistent; if they are not, the operation will have no effect and return INCONSISTENT_POLICY.

* **Parameters:**
  **qos** – [DomainParticipantFactoryQos](domainparticipantfactoryqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactoryQos) to be set. 
* **Returns:**
  RETCODE_IMMUTABLE_POLICY if any of the Qos cannot be changed, RETCODE_INCONSISTENT_POLICY if the Qos is not self consistent and RETCODE_OK if the qos is changed correctly. 

### ReturnCode_t get_library_settings(LibrarySettings &library_settings) const

This operation returns the value of the [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) library settings. 

* **Parameters:**
  **library_settings** – LibrarySettings reference where the settings are returned. 
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_library_settings(const LibrarySettings &library_settings)

This operation sets the library settings. 

Library settings must be set before enabling the DomainParticipants. Otherwise, failure of the setting operation is expected.

* **Parameters:**
  **library_settings** – LibrarySettings to be set. 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if any [DomainParticipant](domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) is already enabled. RETCODE_OK otherwise. 

### ReturnCode_t get_dynamic_type_builder_from_xml_by_name(const std::string &type_name, [DynamicTypeBuilder](../xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE)::\_ref_type &type)

Get the [DynamicType](../xtypes/dynamictype.md#classeprosima_1_1fastdds_1_1dds_1_1DynamicType) defined in XML file. The XML file shall be previously loaded. 

* **Parameters:**
  * **type_name** – Dynamic type name. 
  * **type** – Reference where the Dynamic type builder is returned. 
* **Returns:**
  RETCODE_BAD_PARAMETER if type_name is empty. RETCODE_NO_DATA if type_name is unknown. RETCODE_OK otherwise. 

### xtypes::[ITypeObjectRegistry](../xtypes/typeobjectregistry.md#_CPPv4N8eprosima7fastdds3dds6xtypes19ITypeObjectRegistryE) &type_object_registry()

Return the TypeObjectRegistry member to access the public API. 

* **Returns:**
  const xtypes::TypeObjectRegistry reference. 

### Public Static Functions

### static [DomainParticipantFactory](#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE) \*get_instance()

Returns the [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory) singleton instance.

* **Returns:**
  A raw pointer to the [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory) singleton instance. 

### static std::shared_ptr<[DomainParticipantFactory](#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE)> get_shared_instance()

Returns the [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory) singleton instance.

* **Returns:**
  A shared pointer to the [DomainParticipantFactory](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactory) singleton instance.
