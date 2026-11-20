<a id="api-pim-domainparticipantqos"></a>

<a id="domainparticipantqos"></a>

# DomainParticipantQos

### class DomainParticipantQos

Class [DomainParticipantQos](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos), contains all the possible Qos that can be set for a determined participant. Please consult each of them to check for implementation details and default values. 

Subclassed by [eprosima::fastdds::dds::DomainParticipantExtendedQos](domainparticipantextendedqos.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantExtendedQos)

### Public Types

### using FlowControllerDescriptorList = std::vector<std::shared_ptr<fastdds::rtps::[FlowControllerDescriptor](../../rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptorE)>>

User defined flow controllers to use alongside. 

**Since**
: 2.4.0 

### Public Functions

### inline DomainParticipantQos()

Constructor. 

### inline virtual ~DomainParticipantQos()

Destructor. 

### inline const [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &user_data() const

Getter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Returns:**
  [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy) reference 

### inline [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &user_data()

Getter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Returns:**
  [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy) reference 

### inline void user_data(const [UserDataQosPolicy](../core/policy/userdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds17UserDataQosPolicyE) &value)

Setter for [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

* **Parameters:**
  **value** – [UserDataQosPolicy](../core/policy/userdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1UserDataQosPolicy)

### inline const [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory() const

Getter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

* **Returns:**
  [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) reference 

### inline [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory()

Getter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

* **Returns:**
  [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) reference 

### inline void entity_factory(const [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &value)

Setter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

* **Parameters:**
  **value** – [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

### inline const [ParticipantResourceLimitsQos](../core/policy/participantresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds28ParticipantResourceLimitsQosE) &allocation() const

Getter for ParticipantResourceLimitsQos

* **Returns:**
  ParticipantResourceLimitsQos reference 

### inline [ParticipantResourceLimitsQos](../core/policy/participantresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds28ParticipantResourceLimitsQosE) &allocation()

Getter for ParticipantResourceLimitsQos

* **Returns:**
  ParticipantResourceLimitsQos reference 

### inline void allocation(const [ParticipantResourceLimitsQos](../core/policy/participantresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds28ParticipantResourceLimitsQosE) &allocation)

Setter for ParticipantResourceLimitsQos

* **Parameters:**
  **allocation** – ParticipantResourceLimitsQos 

### inline const [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties() const

Getter for PropertyPolicyQos

* **Returns:**
  PropertyPolicyQos reference 

### inline [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties()

Getter for PropertyPolicyQos

* **Returns:**
  PropertyPolicyQos reference 

### inline void properties(const [PropertyPolicyQos](../core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) &properties)

Setter for PropertyPolicyQos

* **Parameters:**
  **properties** – PropertyPolicyQos 

### inline const [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE) &wire_protocol() const

Getter for [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1WireProtocolConfigQos)

* **Returns:**
  [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1WireProtocolConfigQos) reference 

### inline [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE) &wire_protocol()

Getter for [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1WireProtocolConfigQos)

* **Returns:**
  [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1WireProtocolConfigQos) reference 

### inline void wire_protocol(const [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE) &wire_protocol)

Setter for [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1WireProtocolConfigQos)

* **Parameters:**
  **wire_protocol** – [WireProtocolConfigQos](../core/policy/wireprotocolconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1WireProtocolConfigQos)

### inline const [TransportConfigQos](../core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQosE) &transport() const

Getter for [TransportConfigQos](../core/policy/transportconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1TransportConfigQos)

* **Returns:**
  [TransportConfigQos](../core/policy/transportconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1TransportConfigQos) reference 

### inline [TransportConfigQos](../core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQosE) &transport()

Getter for [TransportConfigQos](../core/policy/transportconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1TransportConfigQos)

* **Returns:**
  [TransportConfigQos](../core/policy/transportconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1TransportConfigQos) reference 

### inline void transport(const [TransportConfigQos](../core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQosE) &transport)

Setter for [TransportConfigQos](../core/policy/transportconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1TransportConfigQos)

* **Parameters:**
  **transport** – [TransportConfigQos](../core/policy/transportconfigqos.md#classeprosima_1_1fastdds_1_1dds_1_1TransportConfigQos)

### inline const fastcdr::string_255 &name() const

Getter for the Participant name

* **Returns:**
  name 

### inline fastcdr::string_255 &name()

Getter for the Participant name

* **Returns:**
  name 

### inline void name(const fastcdr::string_255 &value)

Setter for the Participant name

* **Parameters:**
  **value** – New name to be set 

### inline [FlowControllerDescriptorList](#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos28FlowControllerDescriptorListE) &flow_controllers()

Getter for FlowControllerDescriptorList

* **Returns:**
  FlowControllerDescriptorList reference 

### bool compare_flow_controllers(const [DomainParticipantQos](#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) &qos) const

Compares the flow controllers of two [DomainParticipantQos](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) element-wise.

* **Parameters:**
  **qos** – The [DomainParticipantQos](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantQos) to compare with. 
* **Returns:**
  true if the flow controllers are the same, false otherwise. 

### inline const [FlowControllerDescriptorList](#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQos28FlowControllerDescriptorListE) &flow_controllers() const

Getter for FlowControllerDescriptorList

* **Returns:**
  FlowControllerDescriptorList reference 

### inline rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &builtin_controllers_sender_thread()

Getter for builtin flow controllers sender threads ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &builtin_controllers_sender_thread() const

Getter for builtin flow controllers sender threads ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### void setup_transports(rtps::BuiltinTransports transports, const rtps::BuiltinTransportsOptions &options = rtps::BuiltinTransportsOptions())

Provides a way of easily configuring transport related configuration on certain pre-defined scenarios with certain options.

* **Parameters:**
  * **transports** – Defines the transport configuration scenario to setup. 
  * **options** – Defines the options to be used in the transport configuration. 

### inline void builtin_controllers_sender_thread(const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the builtin flow controllers sender threads ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set 

### inline rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &timed_events_thread()

Getter for timed event ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &timed_events_thread() const

Getter for timed event ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline void timed_events_thread(const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the timed event ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set 

### inline rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &discovery_server_thread()

Getter for discovery server ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &discovery_server_thread() const

Getter for discovery server ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline void discovery_server_thread(const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the discovery server ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set 

### inline rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &typelookup_service_thread()

Getter for TypeLookup service ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &typelookup_service_thread() const

Getter for TypeLookup service ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline void typelookup_service_thread(const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the TypeLookup service ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set 

### inline rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &security_log_thread()

Getter for security log ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &security_log_thread() const

Getter for security log ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline void security_log_thread(const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the security log ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set 

### const [DomainParticipantQos](#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT
