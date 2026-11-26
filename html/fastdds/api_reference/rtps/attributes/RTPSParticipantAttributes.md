<a id="rtpsparticipantattributes"></a>

# RTPSParticipantAttributes

### class RTPSParticipantAttributes

Class [RTPSParticipantAttributes](#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipantAttributes) used to define different aspects of a [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant). 

### Public Functions

### void setup_transports(fastdds::rtps::BuiltinTransports transports, const fastdds::rtps::BuiltinTransportsOptions &options = fastdds::rtps::BuiltinTransportsOptions())

Provides a way of easily configuring transport related configuration on certain pre-defined scenarios with certain options.

* **Parameters:**
  * **transports** – Defines the transport configuration scenario to setup. 
  * **options** – Defines the options to be used in the transport configuration. 

### inline void setName(const char \*nam)

Set the name of the participant. 

### inline const char \*getName() const

Get the name of the participant. 

### Public Members

### LocatorList_t defaultUnicastLocatorList

Default list of Unicast [Locators](../common/Locator/Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators) to be used for any [Endpoint](../Endpoint.md#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) defined inside this [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) in the case that it was defined with NO UnicastLocators. At least ONE locator should be included in this list. 

### LocatorList_t defaultMulticastLocatorList

Default list of Multicast [Locators](../common/Locator/Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators) to be used for any [Endpoint](../Endpoint.md#classeprosima_1_1fastdds_1_1rtps_1_1Endpoint) defined inside this [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) in the case that it was defined with NO MulticastLocators. This is usually left empty. 

### fastdds::rtps::[ExternalLocators](ExternalLocators.md#_CPPv4N8eprosima7fastdds4rtps16ExternalLocatorsE) default_external_unicast_locators

The collection of external locators to use for communication on user created topics. 

### bool ignore_non_matching_locators = false

Whether locators that don’t match with the announced locators should be kept. 

### std::uint32_t sendSocketBufferSize = 0

Send socket buffer size for the send resource. 

Zero value indicates to use default system buffer size. Default value: 0. 

### std::uint32_t listenSocketBufferSize = 0

Listen socket buffer for all listen resources. 

Zero value indicates to use default system buffer size. Default value: 0. 

### fastdds::rtps::NetmaskFilterKind netmaskFilter = fastdds::rtps::NetmaskFilterKind::AUTO

Netmask filter configuration. 

### [GuidPrefix_t](../common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) prefix

Optionally allows user to define the [GuidPrefix_t](../common/GuidPrefix_t/GuidPrefix_t.md#structeprosima_1_1fastdds_1_1rtps_1_1GuidPrefix__t). 

### [BuiltinAttributes](BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributesE) builtin

Builtin parameters. 

### [PortParameters](../common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParametersE) port

Port Parameters. 

### std::vector<[octet](../common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE)> userData

User Data of the participant. 

### std::int32_t participantID = -1

Participant ID. 

### std::string easy_mode_ip = ""

IP of the Host where master Server is located (EASY_MODE context) 

### std::vector<std::shared_ptr<fastdds::rtps::[TransportDescriptorInterface](../../transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE)>> userTransports

User defined transports to use alongside or in place of builtins. 

### bool useBuiltinTransports = true

Set as false to disable the creation of the default transports. 

### [RTPSParticipantAllocationAttributes](RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributesE) allocation

Holds allocation limits affecting collections managed by a participant. 

### [PropertyPolicy](PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) properties

[Property](../common/Property/Property.md#classeprosima_1_1fastdds_1_1rtps_1_1Property) policies. 

### FlowControllerDescriptorList flow_controllers

Flow controllers. 

### fastdds::rtps::[ThreadSettings](ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) builtin_controllers_sender_thread

Thread settings for the builtin flow controllers sender threads. 

### fastdds::rtps::[ThreadSettings](ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) timed_events_thread

Thread settings for the timed events thread. 

### fastdds::rtps::[ThreadSettings](ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) discovery_server_thread

Thread settings for the discovery server thread. 

### fastdds::rtps::[ThreadSettings](ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) typelookup_service_thread

Thread settings for the builtin TypeLookup service requests and replies threads. 

### fastdds::rtps::[ThreadSettings](ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) builtin_transports_reception_threads

Thread settings for the builtin transports reception threads. 

### fastdds::rtps::[ThreadSettings](ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) security_log_thread

Thread settings for the security log thread. 

### std::uint32_t max_msg_size_no_frag = 0

Maximum message size used to avoid fragmentation, set ONLY in LARGE_DATA. 

If this value is not zero, the network factory will allow the initialization of UDP transports with maxMessageSize higher than 65500K.
