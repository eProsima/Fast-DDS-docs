<a id="sendbuffersallocationattributes"></a>

# SendBuffersAllocationAttributes

### struct SendBuffersAllocationAttributes

Holds limits for send buffers allocations. 

### Public Members

### std::size_t preallocated_number = 0u

Initial number of send buffers to allocate.

This attribute controls the initial number of send buffers to be allocated. The default value of 0 will perform an initial guess of the number of buffers required, based on the number of threads from which a send operation could be started. 

### bool dynamic = false

Whether the number of send buffers is allowed to grow.

This attribute controls how the buffer manager behaves when a send buffer is not available. When true, a new buffer will be created. When false, it will wait for a buffer to be returned. This is a trade-off between latency and dynamic allocations. 

### ResourceLimitedContainerConfig network_buffers_config = ResourceLimitedContainerConfig(16u, std::numeric_limits<std::size_t>::maxdummy_avoid_winmax(), 16u)

Configuration for the network buffers.

This attribute controls the allocation behavior of the network buffers used by each send buffer. The default value will use a value of 16 network buffers for both the preallocated buffers and the dynamic increment allocation, with no maximum limit.
