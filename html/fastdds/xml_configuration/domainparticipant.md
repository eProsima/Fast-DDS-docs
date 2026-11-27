<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="participantprofiles"></a>

<a id="domainparticipant-profiles"></a>

# DomainParticipant profiles

The [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) profiles allow the definition of the configuration of [`DomainParticipants`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) through
XML files.
These profiles are defined within the `<participant>` XML tags.

<a id="domainparticipantattributes"></a>

<a id="domainparticipant-xml-attributes"></a>

## DomainParticipant XML attributes

The `<participant>` element has two attributes defined: `profile_name` and `is_default_profile`.

| Name                 | Description                                                                                                                                                                                                                                                                                                                                                                          | Use       |
|----------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| `profile_name`       | Sets the name under which the `<participant>` profile is registered in the DDS Domain,<br/>so that it can be loaded later by the [`DomainParticipantFactory`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE), as shown in<br/>[Loading and applying profiles](making_xml_profiles.md#loadingapplyingprofiles). | Mandatory |
| `is_default_profile` | Sets the `<participant>` profile as the default profile. Thus, if a default profile<br/>exists, it will be used when no other DomainParticipant profile is specified at the<br/>DomainParticipant’s creation.                                                                                                                                                                        | Optional  |

<a id="domainparticipantconfig"></a>

<a id="domainparticipant-configuration"></a>

## DomainParticipant configuration

The `<participant>` element has two child elements: `<domainId>` and `<rtps>`.
All the DomainParticipant configuration options belong to the `<rtps>` element, except for the DDS `DomainId`
which is defined by the `<domainId>` element.
Below a list with the configuration XML elements is presented:

| Name         | Description                                                                                                                                                                                                           | Values                     |   Default |
|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------|-----------|
| `<domainId>` | DomainId to be used by the DomainParticipant.<br/>See [Profile based creation of a DomainParticipant](../dds_layer/domain/domainParticipant/createDomainParticipant.md#dds-layer-domainparticipant-creation-profile). | `uint32_t`                 |         0 |
| `<rtps>`     | *Fast DDS* DomainParticipant configurations.<br/>See [RTPS element type](#rtps).                                                                                                                                      | [RTPS element type](#rtps) |           |

<a id="rtps"></a>

<a id="rtps-element-type"></a>

### RTPS element type

The following is a list with all the possible child XML elements of the `<rtps>` element.
These elements allow the user to define the DomainParticipant configuration.

| Name                                     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Values                                                                                  | Default   |
|------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|-----------|
| `<name>`                                 | The DomainParticipant’s name.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | `string_255`                                                                            |           |
| `<defaultUnicastLocatorList>`            | List of default reception unicast locators<br/>for user data traffic (see<br/>`<metatrafficUnicastLocatorList>`<br/>defined in [Builtin parameters](#builtin)).<br/>It expects a [LocatorListType](common.md#locatorlisttype).                                                                                                                                                                                                                                                                                                                                                                                                                                       | `<locator>`                                                                             |           |
| `<defaultMulticastLocatorList>`          | List of default reception multicast<br/>locators for user data traffic (see<br/>`<metatrafficMulticastLocatorList>`<br/>defined in [Builtin parameters](#builtin)).<br/>It expects a [LocatorListType](common.md#locatorlisttype).                                                                                                                                                                                                                                                                                                                                                                                                                                   | `<locator>`                                                                             |           |
| `<default_external_unicast_locators>`    | List of [External Locators](../transport/announced_locators.md#external-locators)<br/>to announce for the default user traffic of<br/>this participant.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | [ExternalLocatorListType](common.md#externallocatorlisttype)                            |           |
| `<ignore_non_matching_locators>`         | Whether to ignore locators received on<br/>announcements from other participants when<br/>they don’t match with any of the locators<br/>announced by this participant.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | `bool`                                                                                  | false     |
| `<sendSocketBufferSize>`                 | Size in bytes of the send socket buffer.<br/>If the value is zero then *Fast DDS* will<br/>use the system default socket size.<br/>**[Pro]** On TCP transports, it lets the operating<br/>system auto-tune the buffer.                                                                                                                                                                                                                                                                                                                                                                                                                                               | `uint32_t`                                                                              | 0         |
| `<listenSocketBufferSize>`               | Size in bytes of the reception socket<br/>buffer. If the value is zero then<br/>*Fast DDS* will use the system default<br/>socket size.<br/>**[Pro]** On TCP transports, it lets the operating<br/>system auto-tune the buffer.                                                                                                                                                                                                                                                                                                                                                                                                                                      | `uint32_t`                                                                              | 0         |
| `<netmask_filter>`                       | Participant’s netmask<br/>filtering configuration.<br/>See the [Netmask filtering](../transport/interfaces.md#netmask-filtering) section.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | `NetmaskFilterKind`                                                                     | `AUTO`    |
| `<builtin>`                              | [`builtin`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE) public data member of the<br/>[`WireProtocolConfigQos`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE) class.<br/>See the [Builtin parameters](#builtin) section.                                                                                                                                                                                                                                                                                       | [Builtin parameters](#builtin)                                                          |           |
| `<port>`                                 | Allows defining the port and gains related<br/>to the RTPS protocol. See the [Port](#port) section.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | [Port](#port)                                                                           |           |
| `<participantID>`                        | DomainParticipant’s identifier. Typically<br/>it will be automatically generated by the<br/>[DomainParticipantFactory](../dds_layer/domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory).                                                                                                                                                                                                                                                                                                                                                                                                                                 | `int32_t`                                                                               | 0         |
| `<easy_mode_ip>`                         | IP address of the remote discovery server<br/>to connect to using<br/>[Discovery Server Easy Mode](https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | `string`                                                                                | Empty     |
| `<userTransports>`                       | Transport descriptors to be used by the<br/>DomainParticipant. See<br/>[Transport descriptors](transports.md#transportdescriptors).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | `List <string>`                                                                         |           |
| `<useBuiltinTransports>`                 | Boolean field to indicate the system<br/>whether the DomainParticipant will use the<br/>default [`builtin`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE) transports<br/>in addition to its `<userTransports>`.                                                                                                                                                                                                                                                                                                                                                                        | `bool`                                                                                  | true      |
| `<builtinTransports>`                    | Configuration option to determine which transports<br/>will be instantiated if the `useBuiltinTransports` is<br/>set to true. See [Managing the Builtin Transports](../rtps_layer/rtps_layer.md#rtps-layer-builtin-transports).                                                                                                                                                                                                                                                                                                                                                                                                                                      | [BuiltinTransportType](common.md#builtintransporttype)                                  |           |
| `<propertiesPolicy>`                     | Additional configuration properties.<br/>See [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | [PropertiesPolicyType](common.md#propertiespolicytype)                                  |           |
| `<allocation>`                           | Configuration regarding allocation behavior.<br/>It expects a [DomainParticipantAllocationType](#participantallocationtype).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | [DomainParticipantAllocationType](#participantallocationtype)                           |           |
| `<userData>`                             | Additional information attached to the DomainParticipant<br/>and transmitted with the discovery information.<br/>See [UserDataQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#userdataqospolicy).                                                                                                                                                                                                                                                                                                                                                                                                                                                         | `List <string>`                                                                         | Empty     |
| `<prefix>`                               | [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)’s [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) identifies peers<br/>running in the same process. Two participants with identical<br/>8 first bytes on the [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) are considered to be<br/>running in the same process, and therefore intra-process<br/>delivery is used. See [Intra-process delivery](../transport/intraprocess.md#intraprocess-delivery). | `string`                                                                                | Empty     |
| `<builtin_controllers_sender_thread>`    | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the builtin flow controllers sender thread.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)      |           |
| `<timed_events_thread>`                  | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) participant’s timed events thread.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)      |           |
| `<discovery_server_thread>`              | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the discovery server thread.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)      |           |
| `<typelookup_service_thread>`            | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the threads used by the builtin TypeLookup service<br/>to discover unknown remote types.<br/>See [Remote Data Types Discovery](../xtypes/discovery_matching.md#xtypes-discovery-matching).                                                                                                                                                                                                                                                                                                                                                                                    | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)      |           |
| `<builtin_transports_reception_threads>` | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the builtin transports reception threads.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)      |           |
| `<security_log_thread>`                  | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the security log thread.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)      |           |
| `<flow_controller_descriptor_list>`      | Defined flow controller descriptors to be used by the<br/>DomainParticipant. See [Flow Controller Descriptors](common.md#flowcontrollers-xml).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | [FlowControllersQos](../dds_layer/core/policy/eprosimaExtensions.md#flowcontrollersqos) |           |

**Example**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <participant profile_name="domainparticipant_profile_name">
            <domainId>4</domainId>
            <rtps>
                <name>DomainParticipant Name</name>

                <defaultUnicastLocatorList>
                    <!-- LOCATOR_LIST -->
                    <locator>
                        <udpv4>
                            <port>7400</port>
                            <address>192.168.1.41</address>
                        </udpv4>
                    </locator>
                </defaultUnicastLocatorList>

                <defaultMulticastLocatorList>
                    <!-- LOCATOR_LIST -->
                    <locator>
                        <udpv4>
                            <port>7400</port>
                            <address>192.168.2.41</address>
                        </udpv4>
                    </locator>
                </defaultMulticastLocatorList>

                <default_external_unicast_locators>
                    <!-- EXTERNAL_LOCATOR_LIST -->
                    <udpv4 externality="1" cost="0" mask="24">
                        <address>100.100.100.10</address>
                        <port>23456</port>
                    </udpv4>
                </default_external_unicast_locators>

                <ignore_non_matching_locators>true</ignore_non_matching_locators>

                <sendSocketBufferSize>8192</sendSocketBufferSize>

                <listenSocketBufferSize>8192</listenSocketBufferSize>

                <netmask_filter>ON</netmask_filter>

                <builtin>
                    <!-- BUILTIN -->
                </builtin>

                <port>
                    <portBase>7400</portBase>
                    <domainIDGain>200</domainIDGain>
                    <participantIDGain>10</participantIDGain>
                    <offsetd0>0</offsetd0>
                    <offsetd1>1</offsetd1>
                    <offsetd2>2</offsetd2>
                    <offsetd3>3</offsetd3>
                </port>

                <participantID>99</participantID>

                <userTransports>
                    <transport_id>TransportId1</transport_id>
                    <transport_id>TransportId2</transport_id>
                </userTransports>

                <useBuiltinTransports>false</useBuiltinTransports>

                <builtinTransports>DEFAULT</builtinTransports>

                <propertiesPolicy>
                    <!-- PROPERTIES_POLICY -->
                    <properties>
                        <property>
                            <name>Property1Name</name>
                            <value>Property1Value</value>
                            <propagate>false</propagate>
                        </property>
                    </properties>
                </propertiesPolicy>

                <allocation>
                    <!-- ALLOCATION -->
                </allocation>

                <userData>
                    <value>3.4.7.0.C</value>
                </userData>

                <prefix>72.61.73.70.66.61.72.6d.74.65.73.74</prefix>

                <builtin_controllers_sender_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </builtin_controllers_sender_thread>

                <timed_events_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </timed_events_thread>

                <discovery_server_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </discovery_server_thread>

                <typelookup_service_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </typelookup_service_thread>

                <builtin_transports_reception_threads>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </builtin_transports_reception_threads>

                <security_log_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </security_log_thread>

                <flow_controller_descriptor_list>
                    <flow_controller_descriptor>
                        <name>example_flow_controller</name>
                        <scheduler>FIFO</scheduler>
                        <max_bytes_per_period>4096</max_bytes_per_period>
                        <period_ms>500</period_ms>
                        <sender_thread>
                            <scheduling_policy>-1</scheduling_policy>
                            <priority>0</priority>
                            <affinity>0</affinity>
                            <stack_size>-1</stack_size>
                        </sender_thread>
                    </flow_controller_descriptor>
                </flow_controller_descriptor_list>
            </rtps>
        </participant>
    </profiles>
</dds>
```

#### NOTE
- `LOCATOR_LIST` means a [LocatorListType](common.md#locatorlisttype) is expected.
- `EXTERNAL_LOCATOR_LIST` means a [ExternalLocatorListType](common.md#externallocatorlisttype) is expected.
- For `BUILTIN` details, please refer to [Builtin parameters](#builtin).
- `PROPERTIES_POLICY` means that the label is a [PropertiesPolicyType](common.md#propertiespolicytype) block.
- For `ALLOCATION` details, please refer to [ParticipantAllocationType](#participantallocationtype).

<a id="builtin"></a>

<a id="builtin-parameters"></a>

### Builtin parameters

By calling the [`wire_protocol()`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos13wire_protocolEv) member function of the [`DomainParticipantQos`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE),
it is possible to
access the [`builtin`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE) public data member of the [`WireProtocolConfigQos`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE)
class.
This section specifies the available XML members for the configuration of this
[`builtin`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE) parameters.

| Name                                      | Description                                                                                                                                                                                                                                                                                                                                                                                 | Values                                                                           | Default                                                                                                                                                      |
|-------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<discovery_config>`                      | This is the main element within<br/>which discovery-related<br/>settings can be configured.<br/>See [Discovery](../discovery/discovery.md#discovery).                                                                                                                                                                                                                                       | [discovery_config](#dconf)                                                       |                                                                                                                                                              |
| `<avoid_builtin_multicast>`               | Restricts multicast metatraffic<br/>to PDP only.                                                                                                                                                                                                                                                                                                                                            | `bool`                                                                           | `true`                                                                                                                                                       |
| `<use_WriterLivelinessProtocol>`          | Indicates whether to use the<br/>DataWriterLiveliness protocol.                                                                                                                                                                                                                                                                                                                             | `bool`                                                                           | `true`                                                                                                                                                       |
| `<metatrafficUnicastLocatorList>`         | Metatraffic Unicast Locator List.                                                                                                                                                                                                                                                                                                                                                           | A set of `<locator>`  members.  See [LocatorListType](common.md#locatorlisttype) |                                                                                                                                                              |
| `<metatrafficMulticastLocatorList>`       | Metatraffic Multicast Locator List.                                                                                                                                                                                                                                                                                                                                                         | A set of `<locator>`  members.  See [LocatorListType](common.md#locatorlisttype) |                                                                                                                                                              |
| `<initialPeersList>`                      | The list of IP-port address<br/>pairs of all other<br/>[DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) with which<br/>a [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) will<br/>communicate. See<br/>[Initial peers](../discovery/simple.md#simple-initial-peers) | A set of `<locator>`  members.  See [LocatorListType](common.md#locatorlisttype) |                                                                                                                                                              |
| `<metatraffic_external_unicast_locators>` | List of [External Locators](../transport/announced_locators.md#external-locators)<br/>to announce for the metatraffic of<br/>this participant.                                                                                                                                                                                                                                              | [ExternalLocatorListType](common.md#externallocatorlisttype)                     |                                                                                                                                                              |
| `<readerHistoryMemoryPolicy>`             | Memory policy for DataReaders.<br/>See [HistoryQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#historyqospolicykind).                                                                                                                                                                                                                                                        | [HistoryMemoryPolicy](common.md#historymemorypoliciesxml)                        | [`PREALLOCATED`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE) |
| `<writerHistoryMemoryPolicy>`             | Memory policy for DataWriters.<br/>See [HistoryQosPolicyKind](../dds_layer/core/policy/standardQosPolicies.md#historyqospolicykind).                                                                                                                                                                                                                                                        | [HistoryMemoryPolicy](common.md#historymemorypoliciesxml)                        | [`PREALLOCATED`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE) |
| `<readerPayloadSize>`                     | Maximum DataReader’s History<br/>payload size. Allows to reserve<br/>all the required memory at<br/>DataReader initialization.<br/>See [MemoryManagementPolicy](../dds_layer/core/policy/eprosimaExtensions.md#memorymanagementpolicy).                                                                                                                                                     | `uint32_t`                                                                       | 512                                                                                                                                                          |
| `<writerPayloadSize>`                     | Maximum DataWriter’s History<br/>payload size. Allows to reserve<br/>all the required memory at<br/>DataWriter initialization.<br/>See [MemoryManagementPolicy](../dds_layer/core/policy/eprosimaExtensions.md#memorymanagementpolicy).                                                                                                                                                     | `uint32_t`                                                                       | 512                                                                                                                                                          |
| `<mutation_tries>`                        | Number of different ports<br/>to try if DataReader’s physical<br/>port is already in use.                                                                                                                                                                                                                                                                                                   | `uint32_t`                                                                       | 100                                                                                                                                                          |
| `<flow_controller_name>`                  | [FlowControllersQos](../dds_layer/core/policy/eprosimaExtensions.md#flowcontrollersqos) name.                                                                                                                                                                                                                                                                                               | `string`                                                                         | Empty                                                                                                                                                        |

**Example**

```xml
<builtin>
    <discovery_config>
        <discoveryProtocol>CLIENT</discoveryProtocol>

        <discoveryServersList>
            <locator>
                <udpv4>
                    <address>192.168.10.57</address>
                    <port>56542</port>
                </udpv4>
            </locator>
            <locator>
                <udpv4>
                    <address>192.168.10.58</address>
                    <port>24565</port>
                </udpv4>
            </locator>
        </discoveryServersList>

        <ignoreParticipantFlags>FILTER_DIFFERENT_HOST</ignoreParticipantFlags>

        <EDP>SIMPLE</EDP>

        <simpleEDP>
            <PUBWRITER_SUBREADER>true</PUBWRITER_SUBREADER>
            <PUBREADER_SUBWRITER>true</PUBREADER_SUBWRITER>
        </simpleEDP>

        <leaseDuration>
            <!-- DURATION -->
            <sec>20</sec>
        </leaseDuration>

        <leaseAnnouncement>
            <!-- DURATION -->
            <sec>3</sec>
        </leaseAnnouncement>

        <initialAnnouncements>
            <!-- INITIAL_ANNOUNCEMENTS -->
            <count>10</count>
            <period>
                <nanosec>50</nanosec>
            </period>
        </initialAnnouncements>

        <clientAnnouncementPeriod>
            <nanosec>250000000</nanosec>
        </clientAnnouncementPeriod>

        <static_edp_xml_config>file://filename1.xml</static_edp_xml_config>
        <static_edp_xml_config>file://filename2.xml</static_edp_xml_config>
        <static_edp_xml_config>file://filename3.xml</static_edp_xml_config>
    </discovery_config>

    <avoid_builtin_multicast>true</avoid_builtin_multicast>

    <use_WriterLivelinessProtocol>false</use_WriterLivelinessProtocol>

    <metatrafficUnicastLocatorList>
        <!-- LOCATOR_LIST -->
        <locator>
            <udpv4>
                <address>192.168.0.1</address>
            </udpv4>
        </locator>
    </metatrafficUnicastLocatorList>

    <metatrafficMulticastLocatorList>
        <!-- LOCATOR_LIST -->
        <locator>
            <udpv4>
                <address>192.168.0.1</address>
            </udpv4>
        </locator>
    </metatrafficMulticastLocatorList>

    <initialPeersList>
        <!-- LOCATOR_LIST -->
        <locator>
            <udpv4>
                <address>192.168.0.1</address>
            </udpv4>
        </locator>
    </initialPeersList>

    <metatraffic_external_unicast_locators>
        <!-- EXTERNAL_LOCATOR_LIST -->
        <udpv4 externality="1" cost="0" mask="24">
            <address>100.100.100.10</address>
            <port>34567</port>
        </udpv4>
    </metatraffic_external_unicast_locators>

    <readerHistoryMemoryPolicy>PREALLOCATED_WITH_REALLOC</readerHistoryMemoryPolicy>

    <writerHistoryMemoryPolicy>PREALLOCATED_WITH_REALLOC</writerHistoryMemoryPolicy>

    <readerPayloadSize>512</readerPayloadSize>

    <writerPayloadSize>512</writerPayloadSize>

    <mutation_tries>55</mutation_tries>

</builtin>
```

<a id="dconf"></a>

<a id="discovery-config"></a>

#### discovery_config

Through the `<discovery_config>` element, *Fast DDS* allows the configuration of the discovery mechanism via an XML
file.
Please refer to the [Discovery](../discovery/discovery.md#discovery) section for more detail on the various types of discovery mechanisms and
configurable settings.

| Name                                                                                                                                    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Values                                                                                                                     | Default                                                                                                                                          |
|-----------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| `<discoveryProtocol>`                                                                                                                   | Indicates which discovery protocol<br/>the DomainParticipant will use.<br/>See [Discovery mechanisms](../discovery/discovery.md#disc-mechanisms). If not set to<br/>[`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE) or [`NONE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol4NONEE),<br/>`<discoveryServersList>`<br/>element would be used. | [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE) | [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE)                       |
| [`CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6CLIENTE)              |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                            |                                                                                                                                                  |
| [`SERVER`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SERVERE)              |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                            |                                                                                                                                                  |
| [`BACKUP`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6BACKUPE)              |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                            |                                                                                                                                                  |
| [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE) |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                            |                                                                                                                                                  |
| [`NONE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol4NONEE)                  |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                            |                                                                                                                                                  |
| `<discoveryServersList>`                                                                                                                | Describes locators of the servers from which<br/>the DomainParticipant receives discovery<br/>information.<br/>It expects a [LocatorListType](common.md#locatorlisttype).                                                                                                                                                                                                                                                                                                         | `<locator>`                                                                                                                |                                                                                                                                                  |
| `<ignoreParticipantFlags>`                                                                                                              | Restricts metatraffic using several<br/>filtering criteria.<br/>See [Ignore Participant flags](../discovery/general_disc_settings.md#discovery-ignore-flags).                                                                                                                                                                                                                                                                                                                     | [ignoreParticipantFlags](#partfiltering)                                                                                   | [`NO_FILTER`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags9NO_FILTERE) |
| `<EDP>`                                                                                                                                 | If set to [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE), `<simpleEDP>`<br/>element would be used.<br/>If set to `STATIC`, `EDPStatic` will be<br/>performed, configured with the contents<br/>of the XML file set in<br/>`<static_edp_xml_config>`.<br/>See [Discovery](../discovery/discovery.md#discovery).                                                                                        | [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE) | [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE)                       |
| `STATIC`                                                                                                                                |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                            |                                                                                                                                                  |
| `<simpleEDP>`                                                                                                                           | Attributes of the Simple Discovery<br/>Protocol. See [Simple EDP Attributes](../discovery/simple.md#simple-edp-attributes).                                                                                                                                                                                                                                                                                                                                                       | [simpleEDP](#sedp)                                                                                                         |                                                                                                                                                  |
| `<leaseDuration>`                                                                                                                       | Indicates how long the DomainParticipant<br/>should consider remote DomainParticipants<br/>alive.<br/>See [Lease Duration](../discovery/general_disc_settings.md#discovery-lease-dur).                                                                                                                                                                                                                                                                                            | [DurationType](common.md#durationtype)                                                                                     | 20s                                                                                                                                              |
| `<leaseAnnouncement>`                                                                                                                   | The period for the DomainParticipant to<br/>send its discovery message to all other<br/>discovered DomainParticipants as well as<br/>to all Multicast ports.<br/>See [Announcement Period](../discovery/general_disc_settings.md#discovery-lease-announ).                                                                                                                                                                                                                         | [DurationType](common.md#durationtype)                                                                                     | 3s                                                                                                                                               |
| `<initialAnnouncements>`                                                                                                                | Allows the user to configure the number<br/>and period of the DomainParticipant’s initial<br/>discovery messages.<br/>See [Initial Announcements](../discovery/simple.md#initial-announcements).                                                                                                                                                                                                                                                                                  | [Initial Announcements](#initannounce)                                                                                     |                                                                                                                                                  |
| `<clientAnnouncementPeriod>`                                                                                                            | The period for the DomainParticipant to<br/>send its Discovery Message to its servers<br/>and check for EDP endpoints matching.<br/>**[Pro]** adds a property for true send rate<br/>limiting (see [Fine tuning the server send rate limiter](../discovery/discovery_server.md#ds-send-period) ).                                                                                                                                                                                 | [DurationType](common.md#durationtype)                                                                                     | 450 ms                                                                                                                                           |
| `<static_edp_xml_config>`                                                                                                               | The XML filename(s) with the static EDP<br/>configuration. Only necessary if<br/>the `<EDP>` member is set to<br/>`STATIC`. See [STATIC Discovery](../discovery/static.md#discovery-static).                                                                                                                                                                                                                                                                                      | `List <string>`                                                                                                            |                                                                                                                                                  |

<a id="partfiltering"></a>

<a id="ignoreparticipantflags"></a>

##### ignoreParticipantFlags

| Possible values                                                                                                                                                                 | Description                                                           |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------|
| [`NO_FILTER`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags9NO_FILTERE)                                | All Discovery traffic is processed.                                   |
| [`FILTER_DIFFERENT_HOST`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags21FILTER_DIFFERENT_HOSTE)       | Discovery traffic from another host is discarded.                     |
| [`FILTER_DIFFERENT_PROCESS`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags24FILTER_DIFFERENT_PROCESSE) | Discovery traffic from another process on the same host is discarded. |
| [`FILTER_SAME_PROCESS`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags19FILTER_SAME_PROCESSE)           | Discovery traffic from DomainParticipant’s own process is discarded.  |

This option also supports the OR (`|`) operator to filter discovery traffic from other configurations.
For instance, `FILTER_DIFFERENT_PROCESS|FILTER_SAME_PROCESS` value discards discovery traffic from the
DomainParticipant’s own host.

<a id="sedp"></a>

<a id="simpleedp"></a>

##### simpleEDP

| Name                    | Description                                                                                   | Values   | Default   |
|-------------------------|-----------------------------------------------------------------------------------------------|----------|-----------|
| `<PUBWRITER_SUBREADER>` | Indicates if the participant must use<br/>Publication DataWriter and Subscription DataReader. | `bool`   | `true`    |
| `<PUBREADER_SUBWRITER>` | Indicates if the participant must use<br/>Publication DataReader and Subscription DataWriter. | `bool`   | `true`    |

<a id="initannounce"></a>

<a id="initial-announcements"></a>

##### Initial Announcements

| Name       | Description                                                                                                                                                                                                                          | Values                                 | Default   |
|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------|-----------|
| `<count>`  | Number of initial discovery messages to send at the period<br/>specified by<br/>`<period>`. After these announcements, the DomainParticipant will<br/>continue<br/>sending its discovery messages at the `<leaseAnnouncement>` rate. | `uint32_t`                             | 5         |
| `<period>` | The period for the DomainParticipant to send its discovery messages.                                                                                                                                                                 | [DurationType](common.md#durationtype) | 100 ms    |

<a id="port"></a>

<a id="port-configuration"></a>

### Port Configuration

According to the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) (Section 9.6.1.1), the
[`RTPSParticipants`](../api_reference/rtps/participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE)’ discovery traffic unicast listening ports are calculated using the following equation:
$7400 + 250 * DomainId + 10 + 2 * ParticipantId$.
Therefore the following parameters can be specified:

| Name                  | Description                                                                                                                                                     | Values     |   Default |
|-----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|------------|-----------|
| `<portBase>`          | Base `port`.                                                                                                                                                    | `uint16_t` |      7400 |
| `<domainIDGain>`      | Gain in DomainId.                                                                                                                                               | `uint16_t` |       250 |
| `<participantIDGain>` | Gain in [`participant_id`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos14participant_idE). | `uint16_t` |         2 |
| `<offsetd0>`          | Multicast metadata offset.                                                                                                                                      | `uint16_t` |         0 |
| `<offsetd1>`          | Unicast metadata offset.                                                                                                                                        | `uint16_t` |        10 |
| `<offsetd2>`          | Multicast user data offset.                                                                                                                                     | `uint16_t` |         1 |
| `<offsetd3>`          | Unicast user data offset.                                                                                                                                       | `uint16_t` |        11 |

#### WARNING
Changing these default parameters may break compatibility with other RTPS compliant implementations, as well as
with other *Fast DDS* applications with default port settings.

<a id="participantallocationtype"></a>

<a id="id1"></a>

### ParticipantAllocationType

The `ParticipantAllocationType` defines the `<allocation>` element, which allows setting of the parameters
related with the allocation behavior on the DomainParticipant.
Please refer to [ParticipantResourceLimitsQos](../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos) for a detailed documentation on DomainParticipants allocation
configuration.

| Name                   | Description                                                                                                                                                                                                                                                                                   | Values                                                      |
|------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------|
| `<remote_locators>`    | Defines the limits for the remote locators’ collections.<br/>See [RemoteLocatorsAllocationAttributes](../dds_layer/core/policy/eprosimaExtensions.md#remotelocatorsallocationattributes).                                                                                                     | [Remote Locators Allocations](#remote-locators-allocations) |
| `<total_participants>` | DomainParticipant [Allocation Configuration](common.md#commonalloc) to specify the<br/>total number of DomainParticipants in the domain<br/>(local and remote). See<br/>[ResourceLimitedContainerConfig](../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig).      | [Allocation Configuration](common.md#commonalloc)           |
| `<total_readers>`      | DomainParticipant [Allocation Configuration](common.md#commonalloc) to specify the<br/>total number of DataReader on each DomainParticipant<br/>(local and remote). See<br/>[ResourceLimitedContainerConfig](../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig).  | [Allocation Configuration](common.md#commonalloc)           |
| `<total_writers>`      | DomainParticipant [Allocation Configuration](common.md#commonalloc) related to the<br/>total number of DataWriters on each DomainParticipant<br/>(local and remote).<br/>See [ResourceLimitedContainerConfig](../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig). | [Allocation Configuration](common.md#commonalloc)           |
| `<max_partitions>`     | Maximum size of the partitions submessage.<br/>Set to zero for no limit.                                                                                                                                                                                                                      | `uint32_t`                                                  |
| `<max_user_data>`      | Maximum size of the user data submessage.<br/>Set to zero for no limit.                                                                                                                                                                                                                       | `uint32_t`                                                  |
| `<max_properties>`     | Maximum size of the properties submessage.<br/>Set to zero for no limit.                                                                                                                                                                                                                      | `uint32_t`                                                  |
| `<send_buffers>`       | Allocation behaviour for the send buffer<br/>manager.                                                                                                                                                                                                                                         | [Send buffers](#sendbuffers)                                |

**Example**

```xml
<allocation>
    <remote_locators>
        <max_unicast_locators>4</max_unicast_locators>
        <max_multicast_locators>1</max_multicast_locators>
    </remote_locators>

    <total_participants>
        <initial>0</initial>
        <maximum>0</maximum>
        <increment>1</increment>
    </total_participants>

    <total_readers>
        <initial>0</initial>
        <maximum>0</maximum>
        <increment>1</increment>
    </total_readers>

    <total_writers>
        <initial>0</initial>
        <maximum>0</maximum>
        <increment>1</increment>
    </total_writers>

    <max_partitions>256</max_partitions>

    <max_user_data>256</max_user_data>

    <max_properties>512</max_properties>

    <send_buffers>
        <preallocated_number>127</preallocated_number>
        <dynamic>true</dynamic>
        <network_buffers_config>
            <initial>16</initial>
            <maximum>0</maximum>
            <increment>16</increment>
        </network_buffers_config>
    </send_buffers>

    <!-- content_filter cannot be configured using XML (yet) -->
</allocation>
```

<a id="remote-locators-allocations"></a>

<a id="id2"></a>

#### Remote Locators Allocations

| Name                       | Description                                                                                                                                                                                                                                                                                                                                     | Values     |   Default |
|----------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------|-----------|
| `<max_unicast_locators>`   | Maximum number of unicast locators expected on a<br/>remote  entity. It is recommended to use the maximum<br/>number of network interfaces available on the machine<br/>on which DomainParticipant is running.<br/>See [RemoteLocatorsAllocationAttributes](../dds_layer/core/policy/eprosimaExtensions.md#remotelocatorsallocationattributes). | `uint32_t` |         4 |
| `<max_multicast_locators>` | Maximum number of multicast locators expected on a<br/>remote entity. May be set to zero to disable multicast<br/>traffic. See [RemoteLocatorsAllocationAttributes](../dds_layer/core/policy/eprosimaExtensions.md#remotelocatorsallocationattributes).                                                                                         | `uint32_t` |         1 |

<a id="sendbuffers"></a>

<a id="send-buffers"></a>

#### Send buffers

| Name                       | Description                                                                                                                                                                                                                                                                  | Values                                            | Default       |
|----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------|---------------|
| `<preallocated_number>`    | Initial number of send buffers to allocate. See<br/>[SendBuffersAllocationAttributes](../dds_layer/core/policy/eprosimaExtensions.md#sendbuffersallocationattributes).                                                                                                       | `uint32_t`                                        | 0             |
| `<dynamic>`                | Whether the number of send buffers is allowed to<br/>grow. See [SendBuffersAllocationAttributes](../dds_layer/core/policy/eprosimaExtensions.md#sendbuffersallocationattributes).                                                                                            | `bool`                                            | false         |
| `<network_buffers_config>` | Network buffer [Allocation Configuration](common.md#commonalloc) to specify the<br/>number of network buffers to be allocated for each<br/>send buffer. See [ResourceLimitedContainerConfig](../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig). | [Allocation Configuration](common.md#commonalloc) | (16, inf, 16) |

#### NOTE
The default value `0` of `<preallocated_number>` will perform an initial guess of the number of buffers
required, based on the number of threads from which a send operation could be started.
So it does not mean there are no buffers, instead it would use the maximum amount of buffers available.
On the contrary, `<network_buffers_config>` will default to an initial number of 16 buffers, with an infinite
maximum and an increment of 16 buffers per send buffer.
An initial value of `0` will imply more dynamic allocations, especially at the beginning of the execution.
In case of doubt, it should be left to the default values.
