<a id="api-pim-presentationqospolicy"></a>

<a id="presentationqospolicy"></a>

# PresentationQosPolicy

### class PresentationQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies how the samples representing changes to data instances are presented to the subscribing application. This policy affects the application’s ability to specify and receive coherent changes and to see the relative order of changes.access_scope determines the largest scope spanning the entities for which the order and coherency of changes can be preserved. The two booleans control whether coherent access and ordered access are supported within the scope access_scope.

#### NOTE
Immutable Qos Policy 

#### WARNING
This [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) can be defined and is transmitted to the rest of the network but is not implemented in this version.

### Public Functions

### inline PresentationQosPolicy()

Constructor without parameters. 

### virtual ~PresentationQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [PresentationQosPolicyAccessScopeKind](presentationqospolicyaccessscopekind.md#_CPPv4N8eprosima7fastdds3dds36PresentationQosPolicyAccessScopeKindE) access_scope

Access Scope Kind

By default, INSTANCE_PRESENTATION_QOS.

### bool coherent_access

Specifies support coherent access. That is, the ability to group a set of changes as a unit on the publishing end such that they are received as a unit at the subscribing end. by default, false. 

### bool ordered_access

Specifies support for ordered access to the samples received at the subscription end. That is, the ability of the subscriber to see changes in the same order as they occurred on the publishing end. By default, false.
