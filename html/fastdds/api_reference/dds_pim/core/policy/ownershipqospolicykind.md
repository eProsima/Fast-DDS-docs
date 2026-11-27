<a id="api-pim-ownershipqospolicykind"></a>

<a id="ownershipqospolicykind"></a>

# OwnershipQosPolicyKind

### enum eprosima::fastdds::dds::OwnershipQosPolicyKind

Enum OwnershipQosPolicyKind, different kinds of ownership for [OwnershipQosPolicy](ownershipqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipQosPolicy). 

*Values:*

### enumerator SHARED_OWNERSHIP_QOS

Indicates shared ownership for each instance. Multiple writers are allowed to update the same instance and all the updates are made available to the readers. In other words there is no concept of an “owner” for the instances. 

### enumerator EXCLUSIVE_OWNERSHIP_QOS

Indicates each instance can only be owned by one [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter), but the owner of an instance can change dynamically. The selection of the owner is controlled by the setting of the [OwnershipStrengthQosPolicy](ownershipstrengthqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1OwnershipStrengthQosPolicy). The owner is always set to be the highest-strength [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) object among the ones currently “active” (as determined by the [LivelinessQosPolicy](livelinessqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1LivelinessQosPolicy)).
