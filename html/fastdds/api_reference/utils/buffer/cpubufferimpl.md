<a id="api-utils-cpubufferimpl"></a>

<a id="cpubufferimpl-pro"></a>

# CpuBufferImpl **[Pro]**

### template<typename T, typename Allocator = std::allocator<[T](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)>>class CpuBufferImpl : public eprosima::fastdds::[BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<[T](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)>

 **[Pro]**  CPU buffer implementation wrapping std::vector. Provides the reference implementation for CPU memory buffers. 

### Public Functions

### inline std::vector<[T](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE), [Allocator](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)> &get_storage()

Get mutable reference to underlying std::vector. 

### inline const std::vector<[T](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE), [Allocator](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)> &get_storage() const

Get const reference to underlying std::vector. 

### inline virtual std::string get_backend_type() const override

Get the backend type identifier (e.g., `cpu`, `cuda`, `demo`). Each concrete implementation returns its own fixed identifier. 

### inline virtual std::size_t size() const override

Get the number of elements in the buffer. Required by the serialization layer for all backends. 

### inline virtual std::unique_ptr<[CpuBufferImpl](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)<[T](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE), std::allocator<[T](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)>>> to_cpu() const override

Create a CPU copy of this buffer. If already on CPU, may return a copy or the same instance. 

* **Returns:**
  New [BufferImplBase](bufferimplbase.md#classeprosima_1_1fastdds_1_1BufferImplBase) instance on CPU 

### inline virtual std::unique_ptr<[BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<[T](#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)>> clone() const override

Create a deep copy of this buffer. 

* **Returns:**
  New [BufferImplBase](bufferimplbase.md#classeprosima_1_1fastdds_1_1BufferImplBase) instance with copied data
