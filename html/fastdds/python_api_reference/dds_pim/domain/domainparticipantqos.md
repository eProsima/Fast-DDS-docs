<a id="python-api-pim-domainparticipantqos"></a>

<a id="domainparticipantqos"></a>

# DomainParticipantQos

### *class* fastdds.DomainParticipantQos

Class DomainParticipantQos, contains all the possible Qos that can be set for a determined participant.
Please consult each of them to check for implementation details and default values.

#### allocation(\*args)

*Overload 1:*

Getter for ParticipantResourceLimitsQos

* **Return type:**
  `ParticipantResourceLimitsQos`
* **Returns:**
  ParticipantResourceLimitsQos reference

<br/>

*Overload 2:*

Getter for ParticipantResourceLimitsQos

* **Return type:**
  `ParticipantResourceLimitsQos`
* **Returns:**
  ParticipantResourceLimitsQos reference

<br/>

*Overload 3:*

Setter for ParticipantResourceLimitsQos

* **Parameters:**
  **allocation** (`ParticipantResourceLimitsQos`) – ParticipantResourceLimitsQos

#### builtin_controllers_sender_thread(\*args)

*Overload 1:*

Getter for builtin flow controllers sender threads ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 2:*

Getter for builtin flow controllers sender threads ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 3:*

Setter for the builtin flow controllers sender threads ThreadSettings

* **Parameters:**
  **value** (`ThreadSettings`) – New ThreadSettings to be set

#### compare_flow_controllers(qos)

Compares the flow controllers of two DomainParticipantQos element-wise.

* **Parameters:**
  **qos** ([`DomainParticipantQos`](#fastdds.DomainParticipantQos)) – The DomainParticipantQos to compare with.
* **Return type:**
  boolean
* **Returns:**
  true if the flow controllers are the same, false otherwise.

#### discovery_server_thread(\*args)

*Overload 1:*

Getter for discovery server ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 2:*

Getter for discovery server ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 3:*

Setter for the discovery server ThreadSettings

* **Parameters:**
  **value** (`ThreadSettings`) – New ThreadSettings to be set

#### entity_factory(\*args)

*Overload 1:*

Getter for EntityFactoryQosPolicy

* **Return type:**
  [`EntityFactoryQosPolicy`](../core/policy/entityfactoryqospolicy.md#fastdds.EntityFactoryQosPolicy)
* **Returns:**
  EntityFactoryQosPolicy reference

<br/>

*Overload 2:*

Getter for EntityFactoryQosPolicy

* **Return type:**
  [`EntityFactoryQosPolicy`](../core/policy/entityfactoryqospolicy.md#fastdds.EntityFactoryQosPolicy)
* **Returns:**
  EntityFactoryQosPolicy reference

<br/>

*Overload 3:*

Setter for EntityFactoryQosPolicy

* **Parameters:**
  **value** ([`EntityFactoryQosPolicy`](../core/policy/entityfactoryqospolicy.md#fastdds.EntityFactoryQosPolicy)) – EntityFactoryQosPolicy

#### flow_controllers(\*args)

*Overload 1:*

Getter for FlowControllerDescriptorList

* **Return type:**
  `FlowControllerDescriptorList`
* **Returns:**
  FlowControllerDescriptorList reference

<br/>

*Overload 2:*

Getter for FlowControllerDescriptorList

* **Return type:**
  `FlowControllerDescriptorList`
* **Returns:**
  FlowControllerDescriptorList reference

#### name(\*args)

*Overload 1:*

Getter for the Participant name

* **Return type:**
  string
* **Returns:**
  name

<br/>

*Overload 2:*

Setter for the Participant name

* **Parameters:**
  **value** (*string*) – New name to be set.

#### properties(\*args)

*Overload 1:*

Getter for PropertyPolicyQos

* **Return type:**
  `PropertyPolicyQos`
* **Returns:**
  PropertyPolicyQos reference

<br/>

*Overload 2:*

Getter for PropertyPolicyQos

* **Return type:**
  `PropertyPolicyQos`
* **Returns:**
  PropertyPolicyQos reference

<br/>

*Overload 3:*

Setter for PropertyPolicyQos

* **Parameters:**
  **properties** (`PropertyPolicyQos`) – PropertyPolicyQos

#### setup_transports(\*args)

Provides a way of easily configuring transport related configuration on certain pre-defined scenarios with
certain options.

* **Parameters:**
  * **transports** (*eprosima::fastdds::rtps::BuiltinTransports*) – Defines the transport configuration scenario to setup.
  * **options** (*eprosima::fastdds::rtps::BuiltinTransportsOptions* *,* *optional*) – Defines the options to be used in the transport configuration.

#### *property* thisown

The membership flag

#### timed_events_thread(\*args)

*Overload 1:*

Getter for timed event ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 2:*

Getter for timed event ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 3:*

Setter for the timed event ThreadSettings

* **Parameters:**
  **value** (`ThreadSettings`) – New ThreadSettings to be set

#### transport(\*args)

*Overload 1:*

Getter for TransportConfigQos

* **Return type:**
  [`TransportConfigQos`](../core/policy/transportconfigqos.md#fastdds.TransportConfigQos)
* **Returns:**
  TransportConfigQos reference

<br/>

*Overload 2:*

Getter for TransportConfigQos

* **Return type:**
  [`TransportConfigQos`](../core/policy/transportconfigqos.md#fastdds.TransportConfigQos)
* **Returns:**
  TransportConfigQos reference

<br/>

*Overload 3:*

Setter for TransportConfigQos

* **Parameters:**
  **transport** ([`TransportConfigQos`](../core/policy/transportconfigqos.md#fastdds.TransportConfigQos)) – TransportConfigQos

#### typelookup_service_thread(\*args)

*Overload 1:*

Getter for TypeLookup service ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 2:*

Getter for TypeLookup service ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 3:*

Setter for the TypeLookup service ThreadSettings

* **Parameters:**
  **value** (`ThreadSettings`) – New ThreadSettings to be set

#### user_data(\*args)

*Overload 1:*

Getter for UserDataQosPolicy

* **Return type:**
  [`UserDataQosPolicy`](../core/policy/userdataqospolicy.md#fastdds.UserDataQosPolicy)
* **Returns:**
  UserDataQosPolicy reference

<br/>

*Overload 2:*

Getter for UserDataQosPolicy

* **Return type:**
  [`UserDataQosPolicy`](../core/policy/userdataqospolicy.md#fastdds.UserDataQosPolicy)
* **Returns:**
  UserDataQosPolicy reference

<br/>

*Overload 3:*

Setter for UserDataQosPolicy

* **Parameters:**
  **value** ([`UserDataQosPolicy`](../core/policy/userdataqospolicy.md#fastdds.UserDataQosPolicy)) – UserDataQosPolicy

#### wire_protocol(\*args)

*Overload 1:*

Getter for WireProtocolConfigQos

* **Return type:**
  [`WireProtocolConfigQos`](../core/policy/wireprotocolconfigqos.md#fastdds.WireProtocolConfigQos)
* **Returns:**
  WireProtocolConfigQos reference

<br/>

*Overload 2:*

Getter for WireProtocolConfigQos

* **Return type:**
  [`WireProtocolConfigQos`](../core/policy/wireprotocolconfigqos.md#fastdds.WireProtocolConfigQos)
* **Returns:**
  WireProtocolConfigQos reference

<br/>

*Overload 3:*

Setter for WireProtocolConfigQos

* **Parameters:**
  **wire_protocol** ([`WireProtocolConfigQos`](../core/policy/wireprotocolconfigqos.md#fastdds.WireProtocolConfigQos)) – WireProtocolConfigQos

### fastdds.PARTICIPANT_QOS_DEFAULT *= <fastdds.DomainParticipantQos>*

Class DomainParticipantQos, contains all the possible Qos that can be set for a determined participant.
Please consult each of them to check for implementation details and default values.
