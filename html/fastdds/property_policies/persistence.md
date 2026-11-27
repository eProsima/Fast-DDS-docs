<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="property-policies-persistence"></a>

<a id="persistence-service-settings"></a>

# Persistence Service Settings

When using the [Persistence Service](../persistence/persistence.md#persistence-service), the following properties must be set on the
[`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE), or [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) via their [PropertyPolicyQos](../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos).

#### IMPORTANT
It is strongly recommended to define all three properties at the endpoint
([`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) or [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE)) level for predictable behaviour.
`dds.persistence.plugin` and `dds.persistence.sqlite3.filename` are resolved
as a pair from the first level where `dds.persistence.plugin` is present
(endpoint first, then [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE)); a `dds.persistence.sqlite3.filename`
defined at the endpoint level without a `dds.persistence.plugin` at the same level
is ignored.
`dds.persistence.guid` must be defined at the endpoint level; if absent, the behaviour
falls back to [`TRANSIENT_LOCAL_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE), regardless of any plugin configuration.

* Property `dds.persistence.plugin` selects the persistence plugin to use.
  It can be set on the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE), a [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE), or a [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE).
  If this property is absent at the endpoint level, both `dds.persistence.plugin` and
  `dds.persistence.sqlite3.filename` are taken from the [`DomainParticipant`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) instead.
  Setting it at the endpoint level is strongly recommended.
  At the moment, the only valid value is `builtin.SQLITE3`
  (see [PERSISTENCE:SQLITE3 built-in plugin](../persistence/persistence.md#persistence-sqlite3-builtin-plugin)).
* Property `dds.persistence.sqlite3.filename` specifies the path to the SQLite3 database
  file used for persistent storage when the SQLite3 plugin is used.
  This property is optional; the default value is `persistence.db`.
  To avoid undesired delays caused by concurrent access, it is advisable to use a different
  database file for each DataWriter and DataReader.
* Property `dds.persistence.guid` sets the persistence identifier ([`Guid_t`](../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE)) for a
  DataWriter or DataReader.
  This identifier is used to load the appropriate data from the database and to synchronize
  the entity between restarts.
  This property must be defined at the endpoint level.
  If absent, the durability behaviour falls back to [`TRANSIENT_LOCAL_DURABILITY_QOS`](../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE),
  regardless of any plugin configuration.

The following example shows how to configure the persistence service properties along with the
required QoS settings for a correct operation of the service (see [Example](../persistence/persistence.md#persistence-example)).

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
