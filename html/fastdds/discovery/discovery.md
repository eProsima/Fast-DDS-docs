<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="discovery"></a>

<a id="id1"></a>

# Discovery

Fast DDS, as a Data Distribution Service ([DDS](https://www.omg.org/spec/DDS/1.4)) implementation, provides discovery
mechanisms that allow for automatically finding and matching [DataWriters](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReaders](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) across [DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
so they can start sharing data.
This discovery is performed, for all the mechanisms, in two phases.

<a id="disc-phases"></a>

<a id="discovery-phases"></a>

## Discovery phases

1. **Participant Discovery Phase (PDP)**:
   During this phase the [`DomainParticipants`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) acknowledge each other’s existence.
   To do that, each DomainParticipant sends periodic announcement messages, which specify, among other things, unicast
   addresses (IP and port) where the DomainParticipant is listening for incoming meta and user data traffic.
   Two given DomainParticipants will match when they exist in the same DDS Domain.
   By default, the announcement messages are sent using well-known multicast addresses and ports (calculated using the
   `DomainId`).
   Furthermore, it is possible to specify a list of addresses to send
   announcements using unicast (see in [Initial peers](simple.md#simple-initial-peers)).
   Moreover, it is also possible to configure the periodicity of such announcements (see
   [Discovery Configuration](../xml_configuration/domainparticipant.md#dconf)).
2. **Endpoint Discovery Phase (EDP)**:
   During this phase, the [`DataWriters`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) and [`DataReaders`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) acknowledge each other.
   To do that, the DomainParticipants share information about their DataWriters and DataReaders with each other,
   using the communication channels established during the PDP.
   This information contains, among other things, the [`Topic`](../api_reference/dds_pim/topic/topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) and data type (see [Topic](../dds_layer/topic/topic.md#dds-layer-topic)).
   For two endpoints to match, their topic and data type must coincide.
   Once DataWriter and DataReader have matched, they are ready for sending/receiving user data traffic.

#### IMPORTANT
It is possible to use the PDP phase to transmit information about the host, user, and process (physical information)
in which the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) is running.
Please refer to [Physical Data in Discovery Information](../property_policies/non_consolidated_qos.md#property-policies-physical-data) for more information about how to configure the transmitted
physical data.

<a id="disc-mechanisms"></a>

<a id="discovery-mechanisms"></a>

## Discovery mechanisms

Fast DDS provides the following discovery mechanisms:

- [Simple Discovery](simple.md#simple-disc-settings): This is the default mechanism.
  It upholds the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) for both PDP and EDP, and therefore
  provides compatibility with any other DDS and RTPS implementations.
- [Static Discovery](static.md#static-edp): This mechanisms uses the Simple Participant Discovery Protocol (SPDP) for the
  PDP phase (as specified by the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF)), but allows for skipping
  the Simple Endpoint Discovery Protocol (SEDP) phase when all the DataWriters’ and DataReaders’ IPs and ports,
  data types, and Topics are known beforehand.
- [Discovery Server](discovery_server.md#discovery-server): This discovery mechanism uses a centralized discovery architecture,
  where a DomainParticipant, referred as Server, acts as a hub for meta traffic discovery.
- [PDP Relay](pdp_relay.md#pdp-relay) **[Pro]**: This discovery mechanism keeps the DomainParticipants using
  Simple Discovery, and uses a dedicated DomainParticipant that only re-distributes the participant
  announcements it receives, so that they can discover each other without multicast.
- **Manual Discovery**: This mechanism is only compatible with the RTPS layer.
  It disables the PDP, letting the user to manually match and unmatch [`RTPSParticipants`](../api_reference/rtps/participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE), [`RTPSReaders`](../api_reference/rtps/reader/RTPSReader.md#_CPPv4N8eprosima7fastdds4rtps10RTPSReaderE), and
  [`RTPSWriters`](../api_reference/rtps/writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriterE) using whatever external meta-information channel of its choice.
  Therefore, the user must access the RTPSParticipant implemented by the DomainParticipant and directly match the
  RTPS Entities.

<a id="discovery-settings"></a>

<a id="id3"></a>

## Discovery settings

The following sections list and describe the settings available for each of the previously defined discovery mechanisms,
as well as how to define the [DomainParticipantListener](../dds_layer/domain/domainParticipantListener/domainParticipantListener.md#dds-layer-domainparticipantlistener) discovery callbacks.

* [General Discovery Settings](general_disc_settings.md)
  * [Discovery Protocol](general_disc_settings.md#discovery-protocol)
  * [Ignore Participant flags](general_disc_settings.md#ignore-participant-flags)
  * [Lease Duration](general_disc_settings.md#lease-duration)
  * [Announcement Period](general_disc_settings.md#announcement-period)
* [SIMPLE Discovery](simple.md)
  * [Initial Announcements](simple.md#initial-announcements)
  * [Simple EDP Attributes](simple.md#simple-edp-attributes)
  * [Initial peers](simple.md#initial-peers)
* [STATIC Discovery](static.md)
  * [STATIC EDP](static.md#static-edp)
  * [STATIC EDP XML Configuration Specification](static.md#static-edp-xml-configuration-specification)
  * [Checking STATIC EDP XML Files](static.md#checking-static-edp-xml-files)
  * [Loading STATIC EDP XML Files](static.md#loading-static-edp-xml-files)
* [Discovery Server](discovery_server.md)
  * [Discovery Server and Discovery Server Pro](discovery_server.md#discovery-server-and-discovery-server-pro)
  * [Why use a Discovery Server](discovery_server.md#why-use-a-discovery-server)
  * [Does the user data go through the server?](discovery_server.md#does-the-user-data-go-through-the-server)
  * [Quick start](discovery_server.md#quick-start)
  * [Choosing a deployment](discovery_server.md#choosing-a-deployment)
  * [Key concepts](discovery_server.md#key-concepts)
  * [Choosing between Client and Server](discovery_server.md#choosing-between-client-and-server)
  * [The server locator list](discovery_server.md#the-server-locator-list)
  * [Fine tuning discovery server handshake](discovery_server.md#fine-tuning-discovery-server-handshake)
  * [Fine tuning the server send rate limiter <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](discovery_server.md#fine-tuning-the-server-send-rate-limiter-pro)
  * [Topic interest propagation <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](discovery_server.md#topic-interest-propagation-pro)
  * [Fine tuning the topic interest announcements <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](discovery_server.md#fine-tuning-the-topic-interest-announcements-pro)
  * [The GuidPrefix as an optional server unique identifier](discovery_server.md#the-guidprefix-as-an-optional-server-unique-identifier)
  * [Modifying remote servers list at run time](discovery_server.md#modifying-remote-servers-list-at-run-time)
  * [Configure Discovery Server locators using names](discovery_server.md#configure-discovery-server-locators-using-names)
  * [Full example](discovery_server.md#full-example)
  * [Security <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](discovery_server.md#security-pro)
* [PDP Relay <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](pdp_relay.md)
  * [Why use a PDP Relay](pdp_relay.md#why-use-a-pdp-relay)
  * [How it works](pdp_relay.md#how-it-works)
  * [Configuring a PDP Relay](pdp_relay.md#configuring-a-pdp-relay)
  * [Configuring the DomainParticipants that use the relay](pdp_relay.md#configuring-the-domainparticipants-that-use-the-relay)
  * [Launching a PDP Relay with the CLI](pdp_relay.md#launching-a-pdp-relay-with-the-cli)
  * [Troubleshooting](pdp_relay.md#troubleshooting)
* [DomainParticipantListener Discovery Callbacks](disc_callbacks.md)
