<a id="api-pim-rpcentity"></a>

<a id="rpcentity"></a>

# RPCEntity

### class RPCEntity

Abstract base class for all RPC Objects. 

Subclassed by [eprosima::fastdds::dds::rpc::Replier](replier.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier), [eprosima::fastdds::dds::rpc::Requester](requester.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester), [eprosima::fastdds::dds::rpc::Service](service.md#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Service)

### Public Functions

### virtual ReturnCode_t enable() = 0

Enables the entity. 

### virtual ReturnCode_t close() = 0

Disables the entity. 

### virtual bool is_enabled() const = 0

Check if the entity is enabled.
