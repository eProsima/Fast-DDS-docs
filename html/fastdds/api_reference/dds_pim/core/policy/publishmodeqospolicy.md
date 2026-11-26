<a id="api-pim-publishmodeqospolicy"></a>

<a id="publishmodeqospolicy"></a>

# PublishModeQosPolicy

### class PublishModeQosPolicy : public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Class [PublishModeQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1PublishModeQosPolicy), defines the publication mode for a specific writer. 

### Public Functions

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [PublishModeQosPolicyKind](publishmodeqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24PublishModeQosPolicyKindE) kind = SYNCHRONOUS_PUBLISH_MODE

PublishModeQosPolicyKind

By default, SYNCHRONOUS_PUBLISH_MODE.

### std::string flow_controller_name = fastdds::rtps::[FASTDDS_FLOW_CONTROLLER_DEFAULT](../../../rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps31FASTDDS_FLOW_CONTROLLER_DEFAULTE)

Name of the flow controller used when publish mode kind is ASYNCHRONOUS_PUBLISH_MODE. 

**Since**
: 2.4.0
