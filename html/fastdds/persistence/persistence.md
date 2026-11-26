<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="persistence-service"></a>

<a id="id1"></a>

# Persistence Service

Using default QoS, the [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) history is only available for
[DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) throughout the DataWriter’s life.
This means that the history does not persist between DataWriter initializations and therefore it is on an empty
state on DataWriter creation.
Similarly, the DataReader history does not persist the DataReader’s life, thus also being empty on
DataReader creation.
However, *eProsima Fast DDS* offers the possibility to configure the DataWriter’s history to be stored in a
persistent database, so that the DataWriter can load its history from it on creation.
Furthermore, DataReaders can be configured to store the last notified change in the database, so that they can
recover their state on creation.

This mechanism allows recovering a previous state on starting the Data Distribution Service, thus adding robustness to
applications in the case of, for example, unexpected shutdowns.
Configuring the persistence service, DataWriters and DataReaders can resume their operation from the state
in which they were when the shutdown occurred.

#### NOTE
Mind that DataReaders do not store their history into the database, but rather the last notified change from
the DataWriter.
This means that they will resume operation where they left, but they will not have the previous information, since
that was already notified to the application.

#### NOTE
It may be necessary to configure the history depth of the DataWriter and DataReader to a value large enough to
guarantee that all the changes stored in the database are enough for a correct recovery of the application.

#### NOTE
[`RELIABLE_RELIABILITY_QOS`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) reliability is required to allow a full recovery, since it guarantees that all the changes are stored in
the database and that they will be available for the DataWriter/DataReader upon restart.

<a id="persistence-service-conf"></a>

<a id="configuration"></a>

## Configuration

The configuration of the persistence service is accomplished by setting of the appropriate DataWriter and DataReader
[DurabilityQosPolicy](../dds_layer/core/policy/standardQosPolicies.md#durabilityqospolicy), and by specifying the suitable properties for each entity’s ([`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), DataWriter,
or DataReader) [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos).

* For the [Persistence Service](#persistence-service) to have any effect, the [`DurabilityQosPolicyKind`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKindE) needs to be set to
  [`TRANSIENT_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind24TRANSIENT_DURABILITY_QOSE) or [`PERSISTENT_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind25PERSISTENT_DURABILITY_QOSE).
* A persistence identifier ([`Guid_t`](../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE)) must be set on the endpoint ([`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) or
  [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE)) using the property `dds.persistence.guid`.
  This identifier is used to load the appropriate data from the database, and also to synchronize DataWriter and
  DataReader between restarts.
  The GUID consists of 16 bytes separated into two groups:
  * The first 12 bytes correspond to the [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE).
  * The last 4 bytes correspond to the [`EntityId_t`](../api_reference/rtps/common/EntityId_t/EntityId_t.md#_CPPv4N8eprosima7fastdds4rtps10EntityId_tE).

  The persistence identifier is specified using a string of 12 dot-separated bytes, expressed in hexadecimal base,
  followed by a vertical bar separator (`|`) and another 4 dot-separated bytes, also expressed in hexadecimal base
  (see [Example](#persistence-example)).
  For selecting an appropriate GUID for the DataReader and DataWriter, please refer to
  [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) (section *9.3.1 The Globally Unique Identifier (GUID)*).

  This property must be defined at the endpoint level.
  If absent, the durability behaviour falls back to [`TRANSIENT_LOCAL_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE),
  regardless of any plugin configuration.
* A persistence plugin must be configured for managing the database using property `dds.persistence.plugin` (see
  [PERSISTENCE:SQLITE3 built-in plugin](#persistence-sqlite3-builtin-plugin)):

<a id="persistence-sqlite3-builtin-plugin"></a>

<a id="persistence-sqlite3-built-in-plugin"></a>

## PERSISTENCE:SQLITE3 built-in plugin

This plugin provides persistence through a local database file using *SQLite3* API.
To activate the plugin, `dds.persistence.plugin` property must be added to the PropertyPolicyQos of the
DataWriter or DataReader with value `builtin.SQLITE3`.
Optionally, `dds.persistence.sqlite3.filename` property can be added to the entity’s PropertyPolicyQos
to specify the database file name; if omitted, the default value `persistence.db` is used.
These properties are summarized in the following table:

#### **Persistence::SQLITE3 configuration properties**

| Property name                      | Property value                                                                          |
|------------------------------------|-----------------------------------------------------------------------------------------|
| `dds.persistence.plugin`           | `builtin.SQLITE3`                                                                       |
| `dds.persistence.sqlite3.filename` | Name of the file used for persistent storage. <br/><br/>Default value: `persistence.db` |

#### NOTE
To avoid undesired delays caused by concurrent access to the SQLite3 database, it is strongly recommended to
specify a different database file for each DataWriter and DataReader.

#### IMPORTANT
`dds.persistence.plugin` and `dds.persistence.sqlite3.filename` are resolved as a pair:
if `dds.persistence.plugin` is not set at the endpoint (DataWriter/DataReader) level,
both properties are taken from the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) instead; a
`dds.persistence.sqlite3.filename` defined at the endpoint without a
`dds.persistence.plugin` at the same level is ignored.
`dds.persistence.guid` must be defined at the endpoint level; if absent, the behaviour
falls back to [`TRANSIENT_LOCAL_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE), regardless of any plugin configuration.
It is strongly recommended to set all three properties directly on the endpoint for
predictable behaviour.

<a id="persistence-example"></a>

<a id="example"></a>

## Example

This example shows how to configure the persistence service using [PERSISTENCE:SQLITE3 built-in plugin](#persistence-sqlite3-builtin-plugin) plugin both
from C++ and using *eProsima Fast DDS* XML profile files (see [XML profiles](../xml_configuration/xml_configuration.md#xml-profiles)).

### C++

```c++
/*
 * In order for this example to be self-contained, all the entities are created programatically, including the data
 * type and type support. This has been done using Fast DDS Dynamic Types API, but it could be substituted with a
 * Fast DDS-Gen generated type support if an IDL file is available. The Dynamic Type created here is the equivalent
 * of the following IDL:
 *
 *     struct persistence_topic_type
 *     {
 *         unsigned long index;
 *         string message;
 *     };
 */

// Configure persistence service plugin for DomainParticipant
DomainParticipantQos pqos;
pqos.properties().properties().emplace_back("dds.persistence.plugin", "builtin.SQLITE3");
pqos.properties().properties().emplace_back("dds.persistence.sqlite3.filename", "part_persistence_service.db");
DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

/********************************************************************************************************
* CREATE TYPE AND TYPE SUPPORT
*********************************************************************************************************
* This part could be replaced if IDL file and Fast DDS-Gen are available.
* The type is created with name "persistence_topic_type"
* Additionally, create a data object and populate it, just to show how to do it
********************************************************************************************************/
// Create a struct builder for a type with name "persistence_topic_type"
const std::string topic_type_name = "persistence_topic_type";

TypeDescriptor::_ref_type struct_type_descriptor {traits<TypeDescriptor>::make_shared()};
struct_type_descriptor->kind(TK_STRUCTURE);
struct_type_descriptor->name(topic_type_name);
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(struct_type_descriptor)};

// The type consists of two members, and index and a message. Add members to the struct.
MemberDescriptor::_ref_type index_member_descriptor {traits<MemberDescriptor>::make_shared()};
index_member_descriptor->name("index");
index_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                get_primitive_type(TK_UINT32));
struct_builder->add_member(index_member_descriptor);

MemberDescriptor::_ref_type message_member_descriptor {traits<MemberDescriptor>::make_shared()};
message_member_descriptor->name("message");
message_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                create_string_type(static_cast<uint32_t>(LENGTH_UNLIMITED))->build());
struct_builder->add_member(message_member_descriptor);

// Build the type
DynamicType::_ref_type struct_type {struct_builder->build()};

// Create type support and register the type
TypeSupport type_support(new DynamicPubSubType(struct_type));
type_support.register_type(participant);

// Create data sample a populate data. This is to be used when calling `writer->write()`
DynamicData::_ref_type dyn_helloworld {DynamicDataFactory::get_instance()->create_data(struct_type)};

dyn_helloworld->set_uint32_value(0, 0);
dyn_helloworld->set_string_value(1, "HelloWorld");
/********************************************************************************************************
* END CREATE TYPE AND TYPE SUPPORT
********************************************************************************************************/

// Create a topic
Topic* topic = participant->create_topic("persistence_topic_name", topic_type_name, TOPIC_QOS_DEFAULT);

// Create a publisher and a subscriber with default QoS
Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

// Configure DataWriter's durability and persistence GUID so it can use the persistence service
DataWriterQos dwqos = DATAWRITER_QOS_DEFAULT;
dwqos.history().kind = KEEP_LAST_HISTORY_QOS;
dwqos.history().depth = 20;
dwqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
dwqos.durability().kind = TRANSIENT_DURABILITY_QOS;
dwqos.properties().properties().emplace_back("dds.persistence.plugin", "builtin.SQLITE3");
dwqos.properties().properties().emplace_back("dds.persistence.guid",
        "77.72.69.74.65.72.5f.71.65.72.73.5f|67.75.69.64");
dwqos.properties().properties().emplace_back("dds.persistence.sqlite3.filename", "dw_persistence_service.db");
DataWriter* writer = publisher->create_datawriter(topic, dwqos);

// Configure DataReaders's durability and persistence GUID so it can use the persistence service
DataReaderQos drqos = DATAREADER_QOS_DEFAULT;
drqos.history().kind = KEEP_LAST_HISTORY_QOS;
drqos.history().depth = 20;
drqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
drqos.durability().kind = TRANSIENT_DURABILITY_QOS;
drqos.properties().properties().emplace_back("dds.persistence.plugin", "builtin.SQLITE3");
drqos.properties().properties().emplace_back("dds.persistence.guid",
        "77.72.69.74.65.72.5f.71.65.72.73.5f|67.75.69.65");
drqos.properties().properties().emplace_back("dds.persistence.sqlite3.filename", "dr_persistence_service.db");
DataReader* reader = subscriber->create_datareader(topic, drqos);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <!-- DomainParticipant configuration -->
        <participant profile_name="persistence_service_participant">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <!-- Select persistence plugin -->
                        <property>
                            <name>dds.persistence.plugin</name>
                            <value>builtin.SQLITE3</value>
                        </property>
                        <!-- Filename will only be used if endpoint does not define its own -->
                        <property>
                            <name>dds.persistence.sqlite3.filename</name>
                            <value>part_persistence_service.db</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>

        <!-- DataWriter configuration -->
        <data_writer profile_name="persistence_service_data_writer">
            <topic>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                    <depth>20</depth>
                </historyQos>
            </topic>
            <qos>
                <reliability>
                    <kind>RELIABLE</kind>
                </reliability>
                <!-- Set durability to TRANSIENT_DURABILITY_QOS -->
                <durability>
                    <kind>TRANSIENT</kind>
                </durability>
            </qos>
            <propertiesPolicy>
                <properties>
                    <property>
                        <name>dds.persistence.plugin</name>
                        <value>builtin.SQLITE3</value>
                    </property>
                    <!-- Persistence GUID -->
                    <property>
                        <name>dds.persistence.guid</name>
                        <value>77.72.69.74.65.72.5f.71.65.72.73.5f|67.75.69.64</value>
                    </property>
                    <property>
                        <name>dds.persistence.sqlite3.filename</name>
                        <value>dw_persistence_service.db</value>
                    </property>
                </properties>
            </propertiesPolicy>
        </data_writer>

        <!-- DataReader configuration -->
        <data_reader profile_name="persistence_service_data_reader">
            <topic>
                <historyQos>
                    <kind>KEEP_LAST</kind>
                    <depth>20</depth>
                </historyQos>
            </topic>
            <qos>
                <reliability>
                    <kind>RELIABLE</kind>
                </reliability>
                <!-- Set durability to TRANSIENT_DURABILITY_QOS -->
                <durability>
                    <kind>TRANSIENT</kind>
                </durability>
            </qos>
            <propertiesPolicy>
                <properties>
                    <property>
                        <name>dds.persistence.plugin</name>
                        <value>builtin.SQLITE3</value>
                    </property>
                    <!-- Persistence GUID -->
                    <property>
                        <name>dds.persistence.guid</name>
                        <value>77.72.69.74.65.72.5f.71.65.72.73.5f|67.75.69.65</value>
                    </property>
                    <property>
                        <name>dds.persistence.sqlite3.filename</name>
                        <value>dr_persistence_service.db</value>
                    </property>
                </properties>
            </propertiesPolicy>
        </data_reader>
    </profiles>
</dds>
```

#### NOTE
For instructions on how to create DomainParticipants, DataReaders, and DataWriters, please refer to
[Profile based creation of a DomainParticipant](../dds_layer/domain/domainParticipant/createDomainParticipant.md#dds-layer-domainparticipant-creation-profile), [Profile based creation of a DataWriter](../dds_layer/publisher/dataWriter/createDataWriter.md#dds-layer-publisher-datawriter-creation-profile), and
[Profile based creation of a DataReader](../dds_layer/subscriber/dataReader/createDataReader.md#dds-layer-subscriber-datareader-creation-profile) respectively.
