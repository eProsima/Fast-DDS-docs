<a id="api-utils-bufferimplbase"></a>

<a id="bufferimplbase-pro"></a>

# BufferImplBase **[Pro]**

### template<typename T>class BufferImplBase

 **[Pro]**  Abstract base class for all buffer implementations (CPU, CUDA, ROCm, etc.).

This base keeps only what the Buffer<T> `pimpl` and the serialization layer need. Backend-specific APIs (element access, resize, iterators, descriptor serialization, etc.) are the responsibility of each concrete implementation or the [BufferBackend](bufferbackend.md#structeprosima_1_1fastdds_1_1BufferBackend) plugin; the CPU path goes through [CpuBufferImpl](cpubufferimpl.md#classeprosima_1_1fastdds_1_1CpuBufferImpl) directly. 

Subclassed by [eprosima::fastdds::CpuBufferImpl< T, std::allocator< T > >](cpubufferimpl.md#classeprosima_1_1fastdds_1_1CpuBufferImpl), [eprosima::fastdds::CpuBufferImpl< T, Allocator >](cpubufferimpl.md#classeprosima_1_1fastdds_1_1CpuBufferImpl)

### Public Functions

### virtual std::string get_backend_type() const = 0

Get the backend type identifier (e.g., `cpu`, `cuda`, `demo`). Each concrete implementation returns its own fixed identifier. 

### virtual std::size_t size() const = 0

Get the number of elements in the buffer. Required by the serialization layer for all backends. 

### virtual std::unique_ptr<[CpuBufferImpl](cpubufferimpl.md#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)<[T](#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE), std::allocator<[T](#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)>>> to_cpu() const = 0

Create a CPU copy of this buffer. If already on CPU, may return a copy or the same instance. 

* **Returns:**
  New [BufferImplBase](#classeprosima_1_1fastdds_1_1BufferImplBase) instance on CPU 

### virtual std::unique_ptr<[BufferImplBase](#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<[T](#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)>> clone() const = 0

Create a deep copy of this buffer. 

* **Returns:**
  New [BufferImplBase](#classeprosima_1_1fastdds_1_1BufferImplBase) instance with copied data
