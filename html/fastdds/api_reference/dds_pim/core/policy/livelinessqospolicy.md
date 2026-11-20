<a id="api-pim-livelinessqospolicy"></a>

<a id="livelinessqospolicy"></a>

# LivelinessQosPolicy

### class LivelinessQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Determines the mechanism and parameters used by the application to determine whether an [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) is “active” (alive). The “liveliness” status of an [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) is used to maintain instance ownership in combination with the setting of the [OwnershipQosPolicy](ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy). The application is also informed via listener when an [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) is no longer alive.

The [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) requests that liveliness of the writers is maintained by the requested means and loss of liveliness is detected with delay not to exceed the lease_duration.

The [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) commits to signaling its liveliness using the stated means at intervals not to exceed the lease_duration. Listeners are used to notify the DataReaderof loss of liveliness and [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) of violations to the liveliness contract. 

### Public Functions

### inline LivelinessQosPolicy()

Constructor. 

### virtual ~LivelinessQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [LivelinessQosPolicyKind](livelinessqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23LivelinessQosPolicyKindE) kind

Liveliness kind

By default, AUTOMATIC_LIVELINESS.

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) lease_duration

Period within which liveliness should be asserted. 

On a [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) it represents the period it commits to signal its liveliness. On a [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) it represents the period without assertion after which a [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) is considered inactive. By default, [dds::c_TimeInfinite](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1a481d27d32a5fbfe02ca0030ecc4ee6ef). 

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) announcement_period

The period for automatic assertion of liveliness. 

Only used for DataWriters with AUTOMATIC liveliness. By default, [dds::c_TimeInfinite](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1a481d27d32a5fbfe02ca0030ecc4ee6ef).

#### WARNING
When not infinite, must be < lease_duration, and it is advisable to be less than 0.7\*lease_duration.
