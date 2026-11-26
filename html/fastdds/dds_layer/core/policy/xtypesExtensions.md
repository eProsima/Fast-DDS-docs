<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="xtypes-extensions"></a>

<a id="id1"></a>

# XTypes Extensions

This section explain those QoS Policy extensions defined in the [XTypes Specification](https://www.omg.org/spec/DDS-XTypes/):

<a id="datarepresentationqospolicy"></a>

<a id="id2"></a>

## DataRepresentationQosPolicy

This XTypes QoS Policy states which data representations will be used by the [DataWriters](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReaders](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).

The DataWriters offer a single data representation that will be used to communicate with the matched DataReaders.
The DataReaders can request one or more data representations and in order to have communication with the DataWriter,
the offered data representation needs to be contained within the DataReader request.
See [`DataRepresentationQosPolicy`](../../../api_reference/dds_pim/core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicyE).

List of QoS Policy data members:

| Data Member Name                                                                                                                                          | Type                                                       | Default Value   |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------|-----------------|
| [`m_value`](../../../api_reference/dds_pim/core/policy/datarepresentationqospolicy.md#_CPPv4N8eprosima7fastdds3dds27DataRepresentationQosPolicy7m_valueE) | std::vector<[DataRepresentationId](#datarepresentationid)> | Empty list      |

An empty list is interpreted differently depending on the entity:

* [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter): it is equivalent to a list containing only [`XCDR_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId24XCDR_DATA_REPRESENTATIONE).
  The DataWriter offers, and encodes the data with, the first version of the Extended CDR Representation.
* [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader): it is equivalent to a list containing only [`XCDR_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId24XCDR_DATA_REPRESENTATIONE).
  The DataReader only matches DataWriters offering [`XCDR_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId24XCDR_DATA_REPRESENTATIONE).
  In order to match a DataWriter offering [`XCDR2_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId25XCDR2_DATA_REPRESENTATIONE), this value must be explicitly included in
  the list of the DataReader.
* [Topic](../../topic/topic.md#dds-layer-topic): no data representation is defined.
  The value set on a Topic has no effect on its own.
  It is only applied to the DataWriters and DataReaders whose QoS are built from the QoS of the Topic using
  `copy_from_topic_qos()`, in which case an empty list is interpreted as explained above.

#### NOTE
A DataWriter offers a single data representation, so only the first element of its list is taken into account.
The remaining ones are ignored both when encoding the data and when matching with a DataReader.

#### NOTE
This QoS Policy applies to [Topic](../../topic/topic.md#dds-layer-topic), [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) and [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) entities.
<br />
It cannot be changed on enabled entities.

<a id="datarepresentationid"></a>

<a id="id3"></a>

### DataRepresentationId

There are three possible values (see [`DataRepresentationId`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationIdE)):

* [`XCDR_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId24XCDR_DATA_REPRESENTATIONE):
  This option corresponds to the first version of the Extended CDR Representation encoding.
* [`XML_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId23XML_DATA_REPRESENTATIONE):
  This option corresponds to the XML Data Representation.
* [`XCDR2_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId25XCDR2_DATA_REPRESENTATIONE):
  This option corresponds to the second version of the Extended CDR Representation encoding.

#### WARNING
[`XML_DATA_REPRESENTATION`](../../../api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationId23XML_DATA_REPRESENTATIONE) is not supported.
A DataWriter offering it does not match any DataReader, and including it in the list of a DataReader has no
effect.

<a id="example"></a>

### Example

### C++

```c++
// The DataWriter offers XCDR version 2 (only the first element of the list is taken into account)
DataWriterQos writer_qos;
writer_qos.representation().m_value.push_back(DataRepresentationId_t::XCDR2_DATA_REPRESENTATION);
// Use modified QoS in the creation of the corresponding entity
writer_ = publisher_->create_datawriter(topic_, writer_qos);

// The DataReader accepts XCDR version 2 and XCDR version 1
DataReaderQos reader_qos;
reader_qos.representation().m_value.push_back(DataRepresentationId_t::XCDR2_DATA_REPRESENTATION);
reader_qos.representation().m_value.push_back(DataRepresentationId_t::XCDR_DATA_REPRESENTATION);
// Use modified QoS in the creation of the corresponding entity
reader_ = subscriber_->create_datareader(topic_, reader_qos);
```

### XML

```xml
<data_writer profile_name="writer_xml_conf_data_representation_profile">
    <qos>
        <!-- The DataWriter offers XCDR version 2 -->
        <data_representation>
            <value>XCDR2</value>
        </data_representation>
    </qos>
</data_writer>

<data_reader profile_name="reader_xml_conf_data_representation_profile">
    <qos>
        <!-- The DataReader accepts XCDR version 2 and XCDR version 1 -->
        <data_representation>
            <value>XCDR2</value>
            <value>XCDR</value>
        </data_representation>
    </qos>
</data_reader>
```

See [Data Representation](../../../xml_configuration/common.md#xml-datarepresentation)  for the description of the XML elements.

#### NOTE
**[Pro]** Configuring this QoS Policy using XML is only available in *Fast DDS* **[Pro]**.
In *Fast DDS*, it can only be configured using the C++ API.

<a id="typeconsistencyenforcementqospolicy"></a>

<a id="id4"></a>

## TypeConsistencyEnforcementQosPolicy

This XTypes QoS Policy extension defines the rules for determining whether the data type used in the
[DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) is consistent with the one used in the [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
During discovery, a DataReader only matches the DataWriters whose data type is consistent with its own under these
rules.
See [`TypeConsistencyEnforcementQosPolicy`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicyE) and [Type matching](../../../xtypes/discovery_matching.md#xtypes-discovery-matching-type-matching).

#### NOTE
**[Pro]** This QoS Policy is only supported in *Fast DDS* **[Pro]**.
*Fast DDS* ignores it.
A DataReader matches a DataWriter only if both use the same data type or, when either of them does not send its
type information, if both type names are the same.

List of QoS Policy data members:

| Data Member Name                                                                                                                                                                                             | Type                                        | Default Value                                                                                                                                                      |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`m_kind`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy6m_kindE)                                      | [TypeConsistencyKind](#typeconsistencykind) | [`ALLOW_TYPE_COERCION`](../../../api_reference/dds_pim/core/policy/typeconsistencykind.md#_CPPv4N8eprosima7fastdds3dds19TypeConsistencyKind19ALLOW_TYPE_COERCIONE) |
| [`m_ignore_sequence_bounds`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy24m_ignore_sequence_boundsE) | `bool`                                      | `true`                                                                                                                                                             |
| [`m_ignore_string_bounds`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy22m_ignore_string_boundsE)     | `bool`                                      | `true`                                                                                                                                                             |
| [`m_ignore_member_names`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy21m_ignore_member_namesE)       | `bool`                                      | `false`                                                                                                                                                            |
| [`m_prevent_type_widening`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy23m_prevent_type_wideningE)   | `bool`                                      | `false`                                                                                                                                                            |
| [`m_force_type_validation`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy23m_force_type_validationE)   | `bool`                                      | `false`                                                                                                                                                            |
* [`m_kind`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy6m_kindE):
  It determines whether the type in the DataWriter type must be equal to the type in the DataReader or not.
  See [TypeConsistencyKind](#typeconsistencykind) for further details.
  The `ignore_*` data members and [`m_prevent_type_widening`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy23m_prevent_type_wideningE) only apply
  when it is [`ALLOW_TYPE_COERCION`](../../../api_reference/dds_pim/core/policy/typeconsistencykind.md#_CPPv4N8eprosima7fastdds3dds19TypeConsistencyKind19ALLOW_TYPE_COERCIONE).
  [`m_force_type_validation`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy23m_force_type_validationE) applies whatever its value is.
* [`m_ignore_sequence_bounds`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy24m_ignore_sequence_boundsE):
  This data member controls whether the sequence and map bounds are taken into account for type
  assignability or not. If its value is true, the sequences maximum lengths are not considered, which means that a
  sequence T2 with length L2 would be assignable to a sequence T1 with length L1, even if L2 is greater than L1.
  But if it is false, L1 must be higher or equal to L2 to consider the sequences as assignable.
* [`m_ignore_string_bounds`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy22m_ignore_string_boundsE):
  It controls whether the string bounds are considered for type assignation or not.
  If its value is true, the strings maximum lengths are not considered, which means that a string S2 with length L2
  would be assignable to a string S1 with length L1, even if L2 is greater than L1.
  But if it is false, L1 must be higher or equal to L2 to consider the strings as assignable.
* [`m_ignore_member_names`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy21m_ignore_member_namesE):
  This boolean controls whether the member names are taken into consideration for
  type assignability or not.
  If it is false, members are compared by both ID and name: members with the same ID must have the same name, and
  members with the same name must have the same ID.
  If it is true, member names are ignored and only member IDs are compared.
* [`m_prevent_type_widening`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy23m_prevent_type_wideningE):
  This data member controls whether the type widening is allowed or not.
  If it is false, type widening is permitted.
  If it is true, the DataReader’s type cannot be wider than the DataWriter’s type: it cannot have non-optional
  members that are missing from the DataWriter’s type.
* [`m_force_type_validation`](../../../api_reference/dds_pim/core/policy/typeconsistencyenforcementqospolicy.md#_CPPv4N8eprosima7fastdds3dds35TypeConsistencyEnforcementQosPolicy23m_force_type_validationE):
  It controls if the service needs the type information to complete the matching between a
  DataWriter and a DataReader.
  If it is true, a DataReader does not match a DataWriter when either of them does not send its type information.
  If it is false, they match if their type names are the same.

#### NOTE
This QoS Policy applies to [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) entities.
<br />
It cannot be changed on enabled entities.

<a id="id5"></a>

### Example

```c++
// This example only applies to DataReader entities
DataReaderQos reader_qos;
// The TypeConsistencyEnforcementQosPolicy is constructed with kind = ALLOW_TYPE_COERCION by default
// The DataReader type only has to be assignable from the DataWriter type
reader_qos.type_consistency().m_kind = TypeConsistencyKind::ALLOW_TYPE_COERCION;
//Configures the system to take the sequence sizes into account in assignations
reader_qos.type_consistency().m_ignore_sequence_bounds = false;
//Configures the system to take the string sizes into account in assignations
reader_qos.type_consistency().m_ignore_string_bounds = false;
//Configures the system to ignore the member names. Members with same ID could have different names
reader_qos.type_consistency().m_ignore_member_names = true;
//Configures the system to prevent type widening
reader_qos.type_consistency().m_prevent_type_widening = true;
//Configures the system to require the Type Information of both entities in the match process
reader_qos.type_consistency().m_force_type_validation = true;
// Use modified QoS in the creation of the corresponding entity
reader_ = subscriber_->create_datareader(topic_, reader_qos);
```

<a id="typeconsistencykind"></a>

<a id="id6"></a>

### TypeConsistencyKind

There are two possible values:

* [`DISALLOW_TYPE_COERCION`](../../../api_reference/dds_pim/core/policy/typeconsistencykind.md#_CPPv4N8eprosima7fastdds3dds19TypeConsistencyKind22DISALLOW_TYPE_COERCIONE):
  The DataWriter and the DataReader must support the same data type in order to
  communicate.
  Their data types must be equivalent, which is checked on their minimal type representation.
* [`ALLOW_TYPE_COERCION`](../../../api_reference/dds_pim/core/policy/typeconsistencykind.md#_CPPv4N8eprosima7fastdds3dds19TypeConsistencyKind19ALLOW_TYPE_COERCIONE):
  The DataWriter and the DataReader do not need to support the same data type in order to
  communicate as long as the DataReader’s type is assignable from the DataWriter’s type.
