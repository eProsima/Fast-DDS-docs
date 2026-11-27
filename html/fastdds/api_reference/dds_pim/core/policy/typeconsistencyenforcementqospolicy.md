<a id="api-pim-typeconsistencyenforcementqospolicy"></a>

<a id="typeconsistencyenforcementqospolicy"></a>

# TypeConsistencyEnforcementQosPolicy

### class TypeConsistencyEnforcementQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

The [TypeConsistencyEnforcementQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1TypeConsistencyEnforcementQosPolicy) defines the rules for determining whether the type used to publish a given data stream is consistent with that used to subscribe to it. It applies to DataReaders.

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline TypeConsistencyEnforcementQosPolicy()

Constructor. 

### virtual ~TypeConsistencyEnforcementQosPolicy() override = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [TypeConsistencyKind](typeconsistencykind.md#_CPPv4N8eprosima7fastdds3dds19TypeConsistencyKindE) m_kind

TypeConsistencyKind.

By default, ALLOW_TYPE_COERCION.

### bool m_ignore_sequence_bounds

This option controls whether sequence bounds are taken into consideration for type assignability. If the option is set to TRUE, sequence bounds (maximum lengths) are not considered as part of the type assignability. This means that a T2 sequence type with maximum length L2 would be assignable to a T1 sequence type with maximum length L1, even if L2 is greater than L1. If the option is set to false, then sequence bounds are taken into consideration for type assignability and in order for T1 to be assignable from T2 it is required that L1>= L2.

By default, true.

### bool m_ignore_string_bounds

This option controls whether string bounds are taken into consideration for type assignability. If the option is set to TRUE, string bounds (maximum lengths) are not considered as part of the type assignability. This means that a T2 string type with maximum length L2 would be assignable to a T1 string type with maximum length L1, even if L2 is greater than L1. If the option is set to false, then string bounds are taken into consideration for type assignability and in order for T1 to be assignable from T2 it is required that L1>= L2.

By default, true.

### bool m_ignore_member_names

This option controls whether member names are taken into consideration for type assignability. If the option is set to TRUE, member names are considered as part of assignability in addition to member IDs (so that members with the same ID also have the same name). If the option is set to FALSE, then member names are not ignored.

By default, false.

### bool m_prevent_type_widening

This option controls whether type widening is allowed. If the option is set to FALSE, type widening is permitted. If the option is set to TRUE,it shall cause a wider type to not be assignable to a narrower type.

By default, false.

### bool m_force_type_validation

This option requires type information to be available in order to complete matching between a [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) and [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

when set to TRUE, otherwise matching can occur without complete type information when set to FALSE.

By default, false.
