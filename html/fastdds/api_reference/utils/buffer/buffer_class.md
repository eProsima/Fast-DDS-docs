<a id="api-utils-buffer-class"></a>

<a id="buffer-pro"></a>

# Buffer **[Pro]**

### template<typename T, typename Allocator = std::allocator<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE)>>class Buffer

 **[Pro]**  Buffer<T> provides a drop-in replacement for std::vector<T> with support for vendor-specific memory backends (CPU, GPU, etc.).

For CPU backends, it provides implicit conversion to std::vector<T>& for seamless backward compatibility. For non-CPU backends, direct access throws exceptions, requiring explicit conversion via [to_vector()](#classeprosima_1_1fastdds_1_1Buffer_1a62699f6ca8606d9c483831ee0780b7fb). 

### Public Functions

### inline Buffer() noexcept(std::is_nothrow_default_constructible<[CpuBufferImpl](cpubufferimpl.md#_CPPv4I00EN8eprosima7fastdds13CpuBufferImplE)<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE)>>::value)

Default constructor creates CPU buffer. 

### inline explicit Buffer(std::unique_ptr<[BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE)>> impl)

Construct with a backend implementation (set once at construction). This is the only way to set a non-CPU backend; there is no post- construction setter, which avoids race conditions with concurrent reads. 

### inline explicit Buffer(size_type count)

Construct with initial size (CPU backend) 

### inline Buffer(size_type count, const [T](#_CPPv4I00EN8eprosima7fastdds6BufferE) &value)

Construct with initial size and value (CPU backend) 

### inline Buffer(const std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)> &vec)

Construct from std::vector (copy) - for backward compatibility. 

### inline Buffer(std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)> &&vec)

Construct from std::vector (move) - for backward compatibility. 

### inline Buffer(std::initializer_list<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE)> init)

Construct from initializer list - for backward compatibility. 

### inline Buffer(const [Buffer](#_CPPv4N8eprosima7fastdds6Buffer6BufferERK6Buffer) &other)

Copy constructor (deep copy via clone()) 

### inline Buffer([Buffer](#_CPPv4N8eprosima7fastdds6Buffer6BufferERR6Buffer) &&other) noexcept

Move constructor — the moved-from buffer is left in a valid, empty state. 

### inline [Buffer](#_CPPv4I00EN8eprosima7fastdds6BufferE) &operator=(const [Buffer](#_CPPv4I00EN8eprosima7fastdds6BufferE) &other)

Copy assignment (deep copy via clone()) 

### inline [Buffer](#_CPPv4I00EN8eprosima7fastdds6BufferE) &operator=([Buffer](#_CPPv4I00EN8eprosima7fastdds6BufferE) &&other) noexcept

Move assignment — the moved-from buffer is left in a valid, empty state. 

### inline [Buffer](#_CPPv4I00EN8eprosima7fastdds6BufferE) &operator=(std::initializer_list<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE)> init)

Assignment from initializer list - for backward compatibility This must come before vector assignment to resolve ambiguity with {{…}} syntax 

### template<typename U = std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)>, typename std::enable_if<std::is_same<[U](#_CPPv4I0_NSt9enable_ifINSt7is_sameI1UNSt6vectorI1T9AllocatorEEE5valueEiE4typeEEN8eprosima7fastdds6BufferaSER6BufferRK1U), std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)>>::value, int>::type = 0>inline [Buffer](#_CPPv4I00EN8eprosima7fastdds6BufferE) &operator=(const [U](#_CPPv4I0_NSt9enable_ifINSt7is_sameI1UNSt6vectorI1T9AllocatorEEE5valueEiE4typeEEN8eprosima7fastdds6BufferaSER6BufferRK1U) &vec)

Assignment from std::vector (copy) - for backward compatibility Uses SFINAE to avoid ambiguity with initializer lists 

### template<typename U = std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)>, typename std::enable_if<std::is_same<[U](#_CPPv4I0_NSt9enable_ifINSt7is_sameI1UNSt6vectorI1T9AllocatorEEE5valueEiE4typeEEN8eprosima7fastdds6BufferaSER6BufferRR1U), std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)>>::value, int>::type = 0>inline [Buffer](#_CPPv4I00EN8eprosima7fastdds6BufferE) &operator=([U](#_CPPv4I0_NSt9enable_ifINSt7is_sameI1UNSt6vectorI1T9AllocatorEEE5valueEiE4typeEEN8eprosima7fastdds6BufferaSER6BufferRR1U) &&vec)

Assignment from std::vector (move) - for backward compatibility Uses SFINAE to avoid ambiguity with initializer lists 

### inline reference operator[](size_type pos)

Access element at position (CPU only) 

### inline reference at(size_type pos)

Access element with bounds checking (CPU only) 

### inline reference front()

Access first element (CPU only) 

### inline reference back()

Access last element (CPU only) 

### inline pointer data()

Get pointer to data (CPU only) 

### inline bool empty() const

Works for all backends (delegates to [BufferImplBase::size()](bufferimplbase.md#classeprosima_1_1fastdds_1_1BufferImplBase_1a68f0ef233be3a3eaa19600098ce9c465)). 

### inline size_type size() const

Works for all backends (delegates to [BufferImplBase::size()](bufferimplbase.md#classeprosima_1_1fastdds_1_1BufferImplBase_1a68f0ef233be3a3eaa19600098ce9c465)). 

### inline operator std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)>&()

Implicit conversion to std::vector<T, Allocator>& (CPU only). Provides backward compatibility with existing code. 

* **Throws:**
  `std::runtime_error` – if backend is not CPU. 

### inline std::vector<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE), [Allocator](#_CPPv4I00EN8eprosima7fastdds6BufferE)> to_vector() const

Escape hatch: Explicit conversion to std::vector<T, Allocator> (all backends). For non-CPU backends, this triggers a copy to CPU memory. 

* **Returns:**
  A std::vector containing a copy of the buffer data. 

### inline std::string get_backend_type() const

Get the backend type identifier (e.g., `cpu`, `cuda`). Delegates to the underlying implementation — the `impl` is the single source of truth for its own backend type. 

### inline const [BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE)> \*get_impl() const

Get the implementation pointer (read-only). 

### inline [BufferImplBase](bufferimplbase.md#_CPPv4I0EN8eprosima7fastdds14BufferImplBaseE)<[T](#_CPPv4I00EN8eprosima7fastdds6BufferE)> \*get_impl()

Get the implementation pointer (mutable, e.g. for descriptor creation). 

### inline void throw_if_not_cpu_backend() const

Throw exception if not CPU backend. 

* **Throws:**
  `std::runtime_error` – if backend is not CPU.
