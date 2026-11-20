<a id="builtinattributes"></a>

# BuiltinAttributes

### class BuiltinAttributes

Class [BuiltinAttributes](#classeprosima_1_1fastdds_1_1rtps_1_1BuiltinAttributes), to define the behavior of the [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) builtin protocols. 

### Public Functions

### inline void compose(const BuiltinConstantAttributes &builtin_const, const BuiltinMutableAttributes &builtin_mutable)

Composes this object from a BuiltinConstantAttributes and a BuiltinMutableAttributes. Every field is assigned: constant fields are taken from `builtin_const`, mutable fields are taken from `builtin_mutable`. 

* **Parameters:**
  * **builtin_const** – Constant builtin attributes to copy from. 
  * **builtin_mutable** – Mutable builtin attributes to copy from. 

### Public Members

### [DiscoverySettings](DiscoverySettings.md#_CPPv4N8eprosima7fastdds4rtps17DiscoverySettingsE) discovery_config

Discovery protocol related attributes. 

### bool use_WriterLivelinessProtocol = true

Indicates to use the WriterLiveliness protocol. 

### NetworkConfigSet_t network_configuration = 0

Network Configuration. 

### LocatorList_t metatrafficUnicastLocatorList

Metatraffic Unicast Locator List. 

### LocatorList_t metatrafficMulticastLocatorList

Metatraffic Multicast Locator List. 

### fastdds::rtps::[ExternalLocators](ExternalLocators.md#_CPPv4N8eprosima7fastdds4rtps16ExternalLocatorsE) metatraffic_external_unicast_locators

The collection of external locators to use for communication on metatraffic topics. 

### LocatorList_t initialPeersList

Initial peers. 

### MemoryManagementPolicy_t readerHistoryMemoryPolicy = MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE

Memory policy for builtin readers. 

### std::uint32_t readerPayloadSize = BUILTIN_DATA_MAX_SIZE

Maximum payload size for builtin readers. 

### MemoryManagementPolicy_t writerHistoryMemoryPolicy = MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE

Memory policy for builtin writers. 

### std::uint32_t writerPayloadSize = BUILTIN_DATA_MAX_SIZE

Maximum payload size for builtin writers. 

### std::uint32_t mutation_tries = 100u

Mutation tries if the port is being used. 

### bool avoid_builtin_multicast = true

Set to true to avoid multicast traffic on builtin endpoints. 

### std::string flow_controller_name = ""

Flow controller name to use for the builtin writers.
