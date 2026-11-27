<a id="python-api-pim-ownershipqospolicykind"></a>

<a id="ownershipqospolicykind"></a>

# OwnershipQosPolicyKind

### fastdds.SHARED_OWNERSHIP_QOS

Indicates shared ownership for each instance. Multiple writers are allowed to update the same instance and all the
updates are made available to the readers. In other words there is no concept of an “owner” for the instances.

### fastdds.EXCLUSIVE_OWNERSHIP_QOS

Indicates each instance can only be owned by one DataWriter, but the owner of an instance can change dynamically.
The selection of the owner is controlled by the setting of the OwnershipStrengthQosPolicy. The owner is always set
to be the highest-strength DataWriter object among the ones currently “active” (as determined by the LivelinessQosPolicy).
