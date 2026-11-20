<a id="api-pim-wireprotocolconfigqos"></a>

<a id="wireprotocolconfigqos"></a>

# WireProtocolConfigQos

### class WireProtocolConfigQos : public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Qos Policy that configures the wire protocol. 

### Public Functions

### inline WireProtocolConfigQos()

Constructor. 

### virtual ~WireProtocolConfigQos() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### inline ReturnCode_t easy_mode(const std::string &ip)

Setter for ROS 2 Easy Mode IP. 

#### NOTE
The IP address must be an IPv4 address. If it is not, the IP address will not be set.

* **Parameters:**
  **ip** – IP address to set
* **Returns:**
  RETCODE_OK if the IP address is set, an specific error code otherwise: 
* **Returns:**
  RETCODE_BAD_PARAMETER if the IP address is not an IPv4 address. 

### inline const std::string &easy_mode() const

Getter for ROS 2 Easy Mode IP. 

* **Returns:**
  IP address if set, empty string otherwise 

### Public Members

### fastdds::rtps::[GuidPrefix_t](../../../rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) prefix

Optionally allows user to define the GuidPrefix_t. 

### std::int32_t participant_id

Participant ID

By default, -1.

### fastdds::rtps::[BuiltinAttributes](../../../rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributesE) builtin

Builtin parameters. 

### fastdds::rtps::[PortParameters](../../../rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParametersE) port

Port Parameters. 

### rtps::[LocatorList](../../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) default_unicast_locator_list

Default list of Unicast Locators to be used for any Endpoint defined inside this RTPSParticipant in the case that it was defined with NO UnicastLocators. At least ONE locator should be included in this list. 

### rtps::[LocatorList](../../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) default_multicast_locator_list

Default list of Multicast Locators to be used for any Endpoint defined inside this RTPSParticipant in the case that it was defined with NO MulticastLocators. This is usually left empty. 

### rtps::[ExternalLocators](../../../rtps/attributes/ExternalLocators.md#_CPPv4N8eprosima7fastdds4rtps16ExternalLocatorsE) default_external_unicast_locators

The collection of external locators to use for communication on user created topics. 

### bool ignore_non_matching_locators = false

Whether locators that don’t match with the announced locators should be kept.
