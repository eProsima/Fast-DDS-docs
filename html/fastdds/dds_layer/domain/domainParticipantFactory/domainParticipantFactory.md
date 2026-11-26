<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-domainparticipantfactory"></a>

<a id="domainparticipantfactory"></a>

# DomainParticipantFactory

The sole purpose of this class is to allow the creation and destruction of [DomainParticipant](../domainParticipant/domainParticipant.md#dds-layer-domainparticipant) objects.
[`DomainParticipantFactory`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE) itself has no factory, it is a singleton object that can be accessed
through the [`get_instance()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory12get_instanceEv) static member function on the [`DomainParticipantFactory`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE)
class.

The behavior of the DomainParticipantFactory can be modified with the QoS values
specified on [DomainParticipantFactoryQos](#dds-layer-domainparticipantfactoryqos).
Since the DomainParticipantFactory is a singleton, its QoS can only be modified with the
[`DomainParticipantFactory::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory7set_qosERK27DomainParticipantFactoryQos) member function.

DomainParticipantFactory does not accept any Listener, since it is not an Entity.

<a id="dds-layer-domainparticipantfactoryqos"></a>

<a id="domainparticipantfactoryqos"></a>

## DomainParticipantFactoryQos

DomainParticipantFactoryQos controls the behavior of the [DomainParticipantFactory](#dds-layer-domainparticipantfactory).
Internally it contains the following [`QosPolicy`](../../../api_reference/dds_pim/core/policy/qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE) objects:

| QosPolicy class                                                                           | Accessor/Mutator                                                                                                                                                                 | Mutable   |
|-------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------|
| [EntityFactoryQosPolicy](../../core/policy/standardQosPolicies.md#entityfactoryqospolicy) | [`entity_factory()`](../../../api_reference/dds_pim/domain/domainparticipantfactoryqos.md#_CPPv4NK8eprosima7fastdds3dds27DomainParticipantFactoryQos14entity_factoryEv)          | Yes       |
| [ThreadSettings](../../core/policy/eprosimaExtensions.md#threadsettingsqos)               | [`shm_watchdog_thread()`](../../../api_reference/dds_pim/domain/domainparticipantfactoryqos.md#_CPPv4N8eprosima7fastdds3dds27DomainParticipantFactoryQos19shm_watchdog_threadEv) | No        |
| [ThreadSettings](../../core/policy/eprosimaExtensions.md#threadsettingsqos)               | [`file_watch_threads()`](../../../api_reference/dds_pim/domain/domainparticipantfactoryqos.md#_CPPv4N8eprosima7fastdds3dds27DomainParticipantFactoryQos18file_watch_threadsEv)   | No        |

Since the DomainParticipantFactory is a singleton, its QoS can only be modified with the
[`DomainParticipantFactory::set_qos()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory7set_qosERK27DomainParticipantFactoryQos) member function.

```c++
DomainParticipantFactoryQos qos;

// Setting autoenable_created_entities to true makes the created DomainParticipants
// to be enabled upon creation
qos.entity_factory().autoenable_created_entities = true;
if (DomainParticipantFactory::get_instance()->set_qos(qos) != RETCODE_OK)
{
    // Error
    return;
}

// Create a DomainParticipant with the new DomainParticipantFactoryQos.
// The returned DomainParticipant is already enabled
DomainParticipant* enabled_participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == enabled_participant)
{
    // Error
    return;
}

// Setting autoenable_created_entities to false makes the created DomainParticipants
// to be disabled upon creation
qos.entity_factory().autoenable_created_entities = false;
if (DomainParticipantFactory::get_instance()->set_qos(qos) != RETCODE_OK)
{
    // Error
    return;
}

// Create a DomainParticipant with the new DomainParticipantFactoryQos.
// The returned DomainParticipant is disabled and will need to be enabled explicitly
DomainParticipant* disabled_participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == disabled_participant)
{
    // Error
    return;
}
```

<a id="dds-layer-domainparticipantfactory-profiles"></a>

<a id="dds-layer-domainparticipantfactory-load-profiles"></a>

<a id="loading-profiles-from-an-xml-file"></a>

## Loading profiles from an XML file

To create Entities based on XML profiles, the file containing such profiles must be loaded first.

If the profile is described in one of the default loaded files, it will be automatically available on initialization.
Otherwise, [`load_XML_profiles_file()`](../../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory22load_XML_profiles_fileERKNSt6stringE) member function can be used to load the profiles in
the XML.
See section [XML profiles](../../../xml_configuration/xml_configuration.md#xml-profiles) for more information regarding XML profile format and automatic loading.

Once loaded, the name of the profiles can be used to create Entities that will have QoS settings according to
the profile specifications.

```c++
// Load the XML with the profiles
DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

// Profiles can now be used to create Entities
DomainParticipant* participant_with_profile =
        DomainParticipantFactory::get_instance()->create_participant_with_profile(0, "participant_profile");
if (nullptr == participant_with_profile)
{
    // Error
    return;
}
```
