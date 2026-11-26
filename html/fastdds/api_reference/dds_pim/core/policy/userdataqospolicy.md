<a id="api-pim-userdataqospolicy"></a>

<a id="userdataqospolicy"></a>

# UserDataQosPolicy

### class UserDataQosPolicy : public eprosima::fastdds::dds::[GenericDataQosPolicy](genericdataqospolicy.md#_CPPv4N8eprosima7fastdds3dds20GenericDataQosPolicyE)

Class derived from [GenericDataQosPolicy](genericdataqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1GenericDataQosPolicy). 

The purpose of this QoS is to allow the application to attach additional information to the created [Entity](../entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) objects such that when a remote application discovers their existence it can access that information and use it for its own purposes.

One possible use of this QoS is to attach security credentials or some other information that can be used by the remote application to authenticate the source.
