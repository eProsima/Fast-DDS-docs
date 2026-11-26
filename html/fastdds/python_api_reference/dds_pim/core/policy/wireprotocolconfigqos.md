<a id="python-api-pim-wireprotocolconfigqos"></a>

<a id="wireprotocolconfigqos"></a>

# WireProtocolConfigQos

### *class* fastdds.WireProtocolConfigQos

Qos Policy that configures the wire protocol

#### *property* builtin

Builtin parameters.

#### clear()

Clears the QosPolicy object

#### *property* default_external_unicast_locators

The collection of external locators to use for communication on user created topics.

#### *property* default_multicast_locator_list

Default list of Multicast Locators to be used for any Endpoint defined inside this RTPSParticipant in the
case that it was defined with NO MulticastLocators. This is usually left empty.

#### *property* default_unicast_locator_list

Default list of Unicast Locators to be used for any Endpoint defined inside this RTPSParticipant in the case
that it was defined with NO UnicastLocators. At least ONE locator should be included in this list.

#### easy_mode(\*args)

*Overload 1:*

Setter for ROS 2 Easy Mode IP

* **Parameters:**
  **ip** (*string*) – IP address to set

Notes: The IP address must be an IPv4 address. If it is not, the IP address will not be set.

* **Return type:**
  int
* **Returns:**
  RETCODE_OK if the IP address is set, an specific error code otherwise:
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the IP address is not an IPv4 address.

<br/>

*Overload 2:*

Getter for ROS 2 Easy Mode IP

* **Return type:**
  string
* **Returns:**
  IP address if set, empty string otherwise

#### *property* ignore_non_matching_locators

Whether locators that don’t match with the announced locators should be kept.

#### *property* participant_id

Participant ID 
By default, -1.

#### *property* port

Port Parameters

#### *property* prefix

Optionally allows user to define the GuidPrefix_t

#### *property* thisown

The membership flag
