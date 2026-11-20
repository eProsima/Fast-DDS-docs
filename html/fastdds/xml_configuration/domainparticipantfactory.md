<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="participantfactoryprofiles"></a>

<a id="domainparticipantfactory-profiles"></a>

# DomainParticipantFactory profiles

The [DomainParticipantFactory](../dds_layer/domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory) profiles allow the definition of the configuration of [`DomainParticipantFactory`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE)
through XML files.
These profiles are defined within the `<domainparticipant_factory>` XML tags.

<a id="domainparticipantfactoryattributes"></a>

<a id="domainparticipantfactory-xml-attributes"></a>

## DomainParticipantFactory XML attributes

The `<domainparticipant_factory>` element has two attributes defined: `profile_name` and `is_default_profile`.

| Name                 | Description                                                                                                                                                                                                                                                                                                                                                                                        | Use       |
|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| `profile_name`       | Sets the name under which the `<domainparticipant_factory>` profile is registered in the<br/>DDS Domain, so that it can be loaded later by the [`DomainParticipantFactory`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE), as shown in<br/>[Loading and applying profiles](making_xml_profiles.md#loadingapplyingprofiles). | Mandatory |
| `is_default_profile` | Sets the `<domainparticipant_factory>` profile as the default profile. Thus, if a default<br/>profile exists, it will be used when creating the DomainParticipantFactory                                                                                                                                                                                                                           | Optional  |

<a id="domainparticipantfactoryconfig"></a>

<a id="domainparticipantfactory-configuration"></a>

## DomainParticipantFactory configuration

The `<domainparticipant_factory>` element has the following children elements:

| Name    | Description                   | Values                                                 |
|---------|-------------------------------|--------------------------------------------------------|
| `<qos>` | DomainParticipantFactory QoS. | [QoS element type](#domainparticipantfactoryqosconfig) |

<a id="domainparticipantfactoryqosconfig"></a>

<a id="qos-element-type"></a>

### QoS element type

| Name                    | Description                                                                                                                                                                                                                   | Values                                         |
|-------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|
| `<entity_factory>`      | Entity factory QoS Policy.                                                                                                                                                                                                    | [Entity Factory](common.md#xml-entity-factory) |
| `<shm_watchdog_thread>` | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the SHM watchdog thread.<br/>See [Concurrency and multithreading](../library_overview/library_overview.md#concurrency-multithreading). | [ThreadSettings](common.md#threadsettingstype) |
| `<file_watch_threads>`  | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the File watch threads.<br/>See [Concurrency and multithreading](../library_overview/library_overview.md#concurrency-multithreading).  | [ThreadSettings](common.md#threadsettingstype) |

**Example**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <domainparticipant_factory profile_name="domainparticipant_factory_profile_name">
            <qos>
                <entity_factory>
                    <autoenable_created_entities>true</autoenable_created_entities>
                </entity_factory>
                <shm_watchdog_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </shm_watchdog_thread>
                <file_watch_threads>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </file_watch_threads>
            </qos>
        </domainparticipant_factory>
    </profiles>
<dds>
```
