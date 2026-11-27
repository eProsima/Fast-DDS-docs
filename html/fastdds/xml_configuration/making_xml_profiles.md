<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="making-xml-profiles"></a>

<a id="creating-an-xml-profiles-file"></a>

# Creating an XML profiles file

An XML file can contain several XML profiles.
These XML profiles are defined within the `<dds>` element, and in turn, within the `<profiles>` XML elements.
The possible topologies for the definition of XML profiles are specified in [Rooted vs Standalone profiles definition](#rootedvsstandalone).

It is worth mentioning that the first element of the xml profile must have the `xmlns` attribute with the link
`xmlns="http://www.eprosima.com"`, in both rooted or standalone definitions.
That link defines the reference of the `xsd` schema that the `xml` document complies with.

The available profile types are:

* [DomainParticipant profiles](domainparticipant.md#participantprofiles),
* [DataWriter profiles](datawriter.md#publisherprofiles),
* [DataReader profiles](datareader.md#subscriberprofiles),
* [Topic profiles](topic.md#topic),
* [Transport descriptors](transports.md#transportdescriptors),
* [Intra-process delivery profiles](library_settings.md#intra-process-delivery-xml-profile),
* [Log profiles](log.md#logprofiles), and
* [Dynamic Types profiles](dynamic_types.md#xmldynamictypes).

The following sections will show implementation examples for each of these profiles.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <domainparticipant_factory profile_name="domainparticipant_factory_profile">
            <!-- ... -->
        </domainparticipant_factory>

        <participant profile_name="participant_profile">
            <!-- ... -->
        </participant>

        <data_writer profile_name="datawriter_profile">
            <!-- ... -->
        </data_writer>

        <data_reader profile_name="datareader_profile">
            <!-- ... -->
        </data_reader>

        <topic profile_name="topic_profile">
            <!-- ... -->
        </topic>

        <transport_descriptors>
            <!-- ... -->
        </transport_descriptors>
    </profiles>

    <library_settings>
        <!-- ... -->
    </library_settings>

    <log>
        <!-- ... -->
    </log>

    <types>
        <!-- ... -->
    </types>
</dds>
```

#### NOTE
The [Example](example.md#examplexml) section shows an XML file with all the possible configurations and profile types.
This example is useful as a quick reference to look for a particular property and how to use it.
The
[Fast DDS XSD scheme](https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/fastdds_profiles.xsd)
can be used as a quick reference too.

<a id="loadingapplyingprofiles"></a>

<a id="loading-and-applying-profiles"></a>

## Loading and applying profiles

In case the user defines the [`Entity`](../api_reference/dds_pim/core/entity.md#_CPPv4N8eprosima7fastdds3dds6EntityE) profiles via XML files, it is required to load these
XML files using the [`load_XML_profiles_file()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory22load_XML_profiles_fileERKNSt6stringE) public member function before creating any
entity.
It is also possible to load directly the XML information as a string data buffer using the
[`load_XML_profiles_string()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory24load_XML_profiles_stringEPKcNSt6size_tE) public member function.
Moreover, [`create_participant_with_profile()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory31create_participant_with_profileE10DomainId_tRKNSt6stringEP25DomainParticipantListenerRK10StatusMask),
[`create_publisher_with_profile()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant29create_publisher_with_profileERKNSt6stringER12ReturnCode_tP17PublisherListenerRK10StatusMask), [`create_subscriber_with_profile()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant30create_subscriber_with_profileERKNSt6stringER12ReturnCode_tP18SubscriberListenerRK10StatusMask),
[`create_datawriter_with_profile()`](../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher30create_datawriter_with_profileEP5TopicRKNSt6stringEP18DataWriterListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE), and [`create_datareader_with_profile()`](../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber30create_datareader_with_profileEP16TopicDescriptionRKNSt6stringEP18DataReaderListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE)
member functions expect a profile name as an argument.
*Fast DDS* searches the given profile name over all the loaded XML profiles, applying the profile to the entity
if founded.

```cpp
if (RETCODE_OK ==
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("my_profiles.xml"))
{
    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant_with_profile(
        0, "participant_xml_profile");

    Topic* topic =
            participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);

    Publisher* publisher = participant->create_publisher_with_profile("publisher_xml_profile");
    DataWriter* datawriter = publisher->create_datawriter_with_profile(topic, "datawriter_xml_profile");
    Subscriber* subscriber = participant->create_subscriber_with_profile("subscriber_xml_profile");
    DataReader* datareader = subscriber->create_datareader_with_profile(topic, "datareader_xml_profile");
}

// Load XML as string data buffer
std::string xml_profile =
        "\
        <?xml version=\"1.0\" encoding=\"UTF-8\" ?>\
        <dds>\
            <profiles xmlns=\"http://www.eprosima.com\" >\
                <data_writer profile_name=\"test_datawriter_profile\" is_default_profile=\"true\">\
                    <qos>\
                        <durability>\
                            <kind>TRANSIENT_LOCAL</kind>\
                        </durability>\
                    </qos>\
                </data_writer>\
            </profiles>\
        </dds>\
        ";
if (RETCODE_OK ==
        DomainParticipantFactory::get_instance()->load_XML_profiles_string(xml_profile.c_str(),
        xml_profile.length()))
{
    // Create DDS entities with profiles
}
```

For simplicity, the [`create_participant_with_default_profile()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory39create_participant_with_default_profileEv) method takes the default
profile set in the environment to create a participant.
It requires the XML profile to have been already loaded.
Please, refer to [XML profiles](xml_configuration.md#xml-profiles) for further information regarding loading profiles.

#### WARNING
It is worth mentioning that if the same XML profile file is loaded multiple times, the second loading of
the file will result in an error together with the consequent error log.

#### NOTE
To load dynamic types from XML files see the [Loading XML Types profile in Fast DDS application](dynamic_types.md#usage) subsection of [Dynamic Types profiles](dynamic_types.md#xmldynamictypes).

<a id="loading-raw-xml-profiles"></a>

<a id="get-qos-from-raw-xml-profiles"></a>

## Get QoS from raw XML profiles

Although the [standard procedure](#loadingapplyingprofiles) is to first load XML profiles, and then create entities
given the profile name of choice, it is also possible to get the desired QoS from the XML profile directly so it can be
modified before being used.
For this purpose, the following public methods are available:

- [`DomainParticipantFactory`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE)
  - [`get_participant_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4NK8eprosima7fastdds3dds24DomainParticipantFactory28get_participant_qos_from_xmlERKNSt6stringER20DomainParticipantQos)
  - [`get_default_participant_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4NK8eprosima7fastdds3dds24DomainParticipantFactory36get_default_participant_qos_from_xmlERKNSt6stringER20DomainParticipantQos)
  - [`get_participant_extended_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4NK8eprosima7fastdds3dds24DomainParticipantFactory37get_participant_extended_qos_from_xmlERKNSt6stringER28DomainParticipantExtendedQos)
  - [`get_default_participant_extended_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4NK8eprosima7fastdds3dds24DomainParticipantFactory45get_default_participant_extended_qos_from_xmlERKNSt6stringER28DomainParticipantExtendedQos)
- [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE)
  - [`get_publisher_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant26get_publisher_qos_from_xmlERKNSt6stringER12PublisherQos)
  - [`get_default_publisher_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant34get_default_publisher_qos_from_xmlERKNSt6stringER12PublisherQos)
  - [`get_subscriber_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant27get_subscriber_qos_from_xmlERKNSt6stringER13SubscriberQos)
  - [`get_default_subscriber_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant35get_default_subscriber_qos_from_xmlERKNSt6stringER13SubscriberQos)
  - [`get_topic_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant22get_topic_qos_from_xmlERKNSt6stringER8TopicQos)
  - [`get_default_topic_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant30get_default_topic_qos_from_xmlERKNSt6stringER8TopicQos)
  - [`get_requester_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant26get_requester_qos_from_xmlERKNSt6stringER12RequesterQos)
  - [`get_default_requester_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant34get_default_requester_qos_from_xmlERKNSt6stringER12RequesterQos)
  - [`get_replier_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant24get_replier_qos_from_xmlERKNSt6stringER10ReplierQos)
  - [`get_default_replier_qos_from_xml()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant32get_default_replier_qos_from_xmlERKNSt6stringER10ReplierQos)
- [`Publisher`](../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE)
  - [`get_datawriter_qos_from_xml()`](../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4NK8eprosima7fastdds3dds9Publisher27get_datawriter_qos_from_xmlERKNSt6stringER13DataWriterQos)
  - [`get_default_datawriter_qos_from_xml()`](../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4NK8eprosima7fastdds3dds9Publisher35get_default_datawriter_qos_from_xmlERKNSt6stringER13DataWriterQos)
- [`Subscriber`](../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE)
  - [`get_datareader_qos_from_xml()`](../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4NK8eprosima7fastdds3dds10Subscriber27get_datareader_qos_from_xmlERKNSt6stringER13DataReaderQos)
  - [`get_default_datareader_qos_from_xml()`](../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4NK8eprosima7fastdds3dds10Subscriber35get_default_datareader_qos_from_xmlERKNSt6stringER13DataReaderQos)

For each `qos` kind there exists two method versions; with and without `profile_name` argument.
When provided, the method searches for the given profile name in the loaded XML profiles,
and results in error if not found.
When not provided, the method fills the provided `qos` object with the first retrieved profile of the
pertinent kind, and results in error when none found.
In addition, there exists another method version which looks for the default profile of the pertinent kind
in the provided XML string.

Following is an example of how to get the QoS from a raw XML profile and modify it before creating a new entity.

```cpp
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);

Topic* topic =
        participant->create_topic("TopicName", "DataTypeName", TOPIC_QOS_DEFAULT);

Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);

// Load XML as string data buffer
std::string xml_profile =
        "\
        <?xml version=\"1.0\" encoding=\"UTF-8\" ?>\
        <dds>\
            <profiles xmlns=\"http://www.eprosima.com\" >\
                <data_writer profile_name=\"test_datawriter_profile\" is_default_profile=\"true\">\
                    <qos>\
                        <durability>\
                            <kind>TRANSIENT_LOCAL</kind>\
                        </durability>\
                    </qos>\
                </data_writer>\
            </profiles>\
        </dds>\
        ";

// Extract Qos from XML
DataWriterQos qos;
if (RETCODE_OK == publisher->get_datawriter_qos_from_xml(xml_profile, qos, "test_datawriter_profile"))
{
    // Modify extracted qos and use it to create DDS entities
}
```

<a id="rootedvsstandalone"></a>

<a id="rooted-vs-standalone-profiles-definition"></a>

## Rooted vs Standalone profiles definition

*Fast DDS* offers various options for the definition of XML profiles. These options are:

* Stand-alone:
  The element defining the XML profile is the root element of the XML file.
  Elements `<dds>`, `<profiles>`, `<library_settings>`, `<types>`, and `<log>` can be defined in a
  stand-alone manner.
* Rooted:
  The element defining the XML profile is the child element of another element.
  For example, the `<participant>`, `<data_reader>`, `<data_writer>`, `<topic>`, and
  `<transport_descriptors>` elements must be defined as child elements of the `<profiles>` element.

The following is an example of the definition of the `<types>` XML profile using the two previously discussed
approaches.

| **Stand-alone**                                                                                                                                                                                                                                                                                                                                                |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ```xml<br/><?xml version="1.0" encoding="UTF-8" ?><br/><types xmlns="http://www.eprosima.com"><br/>    <type><br/>        <!-- Type definition --><br/>    </type><br/><br/>    <type><br/>        <!-- Type definition --><br/>        <!-- Type definition --><br/>    </type><br/></types><br/>```                                                          |
| **Rooted**                                                                                                                                                                                                                                                                                                                                                     |
| ```xml<br/><?xml version="1.0" encoding="UTF-8" ?><br/><dds xmlns="http://www.eprosima.com"><br/>    <types><br/>        <type><br/>            <!-- Type definition --><br/>        </type><br/><br/>        <type><br/>            <!-- Type definition --><br/>            <!-- Type definition --><br/>        </type><br/>    </types><br/></dds><br/>``` |

#### NOTE
Make sure that the first element of the xml profile must have the `xmlns` tag with the link
`xmlns="http://www.eprosima.com"`, in both rooted or standalone definitions.

<a id="modifying-predefined-xml-profiles"></a>

## Modifying predefined XML profiles

Some scenarios may require to modify some of the QoS after loading the XML profiles.
For such cases the [Types of Entities](../dds_layer/core/entity/entity.md#dds-layer-core-entity-types) which act as factories provide methods to get the QoS from the
XML profile. This allows the user to read and modify predefined XML profiles before applying them to a new entity.

```cpp
if (RETCODE_OK ==
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("my_profiles.xml"))
{
    DomainParticipantQos participant_qos;
    DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
        "participant_xml_profile",
        participant_qos);

    // Name obtained in another section of the code
    participant_qos.name() = custom_name;

    // Modify number of preallocations (this overrides the one set in the XML profile)
    participant_qos.allocation().send_buffers.preallocated_number = 10;

    // Create participant using the modified XML Qos
    DomainParticipant* participant =
            DomainParticipantFactory::get_instance()->create_participant(
        0, participant_qos);
}
```

<a id="xml-environment-variables"></a>

<a id="dynamic-content-by-leveraging-environment-variables"></a>

## Dynamic content by leveraging environment variables

For deployment scenarios that require part of the XML content to be dynamically generated, Fast DDS supports using
environment variables on the text content of any XML tag.
The format for environment variables expansion is `${ENV_VAR_NAME}`, where `ENV_VAR_NAME` follows the restrictions
from [IEEE 1003.1](https://pubs.opengroup.org/onlinepubs/000095399/basedefs/xbd_chap08.html):

#### NOTE
Environment variable names … consist solely of uppercase letters, digits, and the ‘_’ (underscore) from the
characters defined in Portable Character Set and do not begin with a digit.

More than one environment variable can be used, and they can be mixed with literal text.

The expansion will take place when the XML file is loaded, so changing the value of an environment variable afterwards
will have no effect.

The following is an example of an XML allowing a participant to exclusively communicate with the participants on a
fixed IP address, taken from `REMOTE_IP_ADDRESS` environment variable.
It also gives the participant a name that mixes literal text with the content from two environment variables.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="env_var_parsing_example">
            <rtps>
                <name>This is app '${MY_APP_NAME}' running on host '${MY_HOST_NAME}'</name>
                <builtin>
                    <initialPeersList>
                        <locator>
                            <udpv4>
                                <address>${REMOTE_IP_ADDRESS}</address>
                            </udpv4>
                        </locator>
                    </initialPeersList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

#### WARNING
The [Fast DDS XSD schema](https://github.com/eProsima/Fast-DDS/blob/master/resources/xsd/fastdds_profiles.xsd)
does not support the environment variables expansion feature, so validation of an XML file with environment
variables expansion expressions will fail.
