<a id="discoverysettings"></a>

# DiscoverySettings

### class DiscoverySettings

Class [DiscoverySettings](#classeprosima_1_1fastdds_1_1rtps_1_1DiscoverySettings), to define the attributes of the several discovery protocols available 

### Public Functions

### inline void static_edp_xml_config(const char \*str)

Set the static endpoint XML configuration. 

* **Parameters:**
  **str** – URI specifying the static endpoint XML configuration. The string could contain a filename ([file://](file://)) or the XML content directly (data://). 

### inline const char \*static_edp_xml_config() const

Get the static endpoint XML configuration. 

* **Returns:**
  URI specifying the static endpoint XML configuration. The string could contain a filename ([file://](file://)) or the XML content directly (data://). 

### Public Members

### [DiscoveryProtocol](DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocolE) discoveryProtocol = [DiscoveryProtocol](DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocolE)::[SIMPLE](DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE)

Chosen discovery protocol. 

### bool use_SIMPLE_EndpointDiscoveryProtocol = true

If set to true, SimpleEDP would be used. 

### bool use_STATIC_EndpointDiscoveryProtocol = false

If set to true, StaticEDP based on an XML file would be implemented. The XML filename must be provided. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) leaseDuration = {20, 0}

Lease Duration of the [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant), indicating how much time remote RTPSParticipants should consider this [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) alive. 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) leaseDuration_announcementperiod = {3, 0}

The period for the [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) to send its Discovery Message to all other discovered RTPSParticipants as well as to all Multicast ports. 

### [InitialAnnouncementConfig](InitialAnnouncementConfig.md#_CPPv4N8eprosima7fastdds4rtps25InitialAnnouncementConfigE) initial_announcements

Initial announcements configuration. 

### [SimpleEDPAttributes](SimpleEDPAttributes.md#_CPPv4N8eprosima7fastdds4rtps19SimpleEDPAttributesE) m_simpleEDP

Attributes of the SimpleEDP protocol. 

### PDPFactory m_PDPfactory = {}

function that returns a PDP object (only if EXTERNAL selected) 

### dds::[Duration_t](../../dds_pim/core/Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) discoveryServer_client_syncperiod = {0, 450 \* 1000000}

The period for the [RTPSParticipant](../participant/RTPSParticipant.md#classeprosima_1_1fastdds_1_1rtps_1_1RTPSParticipant) to: send its Discovery Message to its servers check for EDP endpoints matching 

### eprosima::fastdds::rtps::[LocatorList](../common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) m_DiscoveryServers

Discovery Server initial connections, needed if `discoveryProtocol` = CLIENT | SUPER_CLIENT | SERVER | BACKUP. 

### [ParticipantFilteringFlags](ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlagsE) ignoreParticipantFlags = [ParticipantFilteringFlags](ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlagsE)::[NO_FILTER](ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags9NO_FILTERE)

Filtering participants out depending on location.
