<a id="api-pim-loanablesequence"></a>

<a id="loanablesequence"></a>

# LoanableSequence

### template<typename T, typename \_NonConstEnabler = std::true_type>class LoanableSequence : public eprosima::fastdds::dds::LoanableTypedCollection<[T](#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE), std::true_type>

A type-safe, ordered collection of elements that can receive the buffer from outside (loan).

For users who define data types in OMG IDL, this type corresponds to the IDL express sequence<T>.

For any user-data type Foo that an application defines for the purpose of data-distribution with Fast DDS, a ‘`using FooSeq = LoanableSequence<Foo>`’ is generated. The sequence offers a subset of the methods defined by the standard OMG IDL to C++ mapping for sequences. We refer to an IDL ‘`sequence<Foo>`’ as `FooSeq`.

The state of a sequence is described by the properties ‘maximum’, ‘length’ and ‘has_ownership’. 
* The ‘maximum’ represents the size of the underlying buffer; this is the maximum number of elements it can possibly hold. It is returned by the [maximum()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) operation. 
* The ‘length’ represents the actual number of elements it currently holds. It is returned by the [length()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) operation. 
* The ‘has_ownership’ flag represents whether the sequence owns the underlying buffer. It is returned by the [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) operation. If the sequence does not own the underlying buffer, the underlying buffer is loaned from somewhere else. This flag influences the lifecycle of the sequence and what operations are allowed on it. The general guidelines are provided below and more details are described in detail as pre-conditions and post-conditions of each of the sequence’s operations:

* If has_ownership == true, the sequence has ownership on the buffer. It is then responsible for destroying the buffer when the sequence is destroyed. 
* If has_ownership == false, the sequence does not have ownership on the buffer. This implies that the sequence is loaning the buffer. The sequence should not be destroyed until the loan is returned. 
* A sequence with a zero maximum always has has_ownership == true 

### Public Functions

### LoanableSequence() = default

Default constructor.

Creates the sequence with no data.

* **Post:**
  [buffer()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1ad025ab06796dabef67b51be8ee372420) == nullptr 
* **Post:**
  [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true 
* **Post:**
  [length()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == 0 
* **Post:**
  [maximum()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == 0 

### inline LoanableSequence(size_type max)

Pre-allocation constructor.

Creates the sequence with an initial number of allocated elements. When the input parameter is less than or equal to 0, the behavior is equivalent to the default constructor. Otherwise, the post-conditions below will apply.

* **Parameters:**
  **max** – **[in]** Number of elements to pre-allocate.
* **Post:**
  [buffer()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1ad025ab06796dabef67b51be8ee372420) != nullptr 
* **Post:**
  [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true 
* **Post:**
  [length()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == 0 
* **Post:**
  [maximum()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == max 

### inline ~LoanableSequence()

Deallocate this sequence’s buffer.

* **Pre:**
  [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true. If this precondition is not met, no memory will be released and a warning will be logged. 
* **Post:**
  [maximum()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == 0 and the underlying buffer is released. 

### inline LoanableSequence(const [LoanableSequence](#_CPPv4N8eprosima7fastdds3dds16LoanableSequence16LoanableSequenceERK16LoanableSequence) &other)

Construct a sequence with the contents of another sequence.

This method performs a deep copy of the sequence received into this one. Allocations will happen when other.length() > 0

* **Parameters:**
  **other** – **[in]** The sequence from where contents are to be copied.
* **Post:**
  [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true 
* **Post:**
  [maximum()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == other.length() 
* **Post:**
  [length()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == other.length() 
* **Post:**
  [buffer()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1ad025ab06796dabef67b51be8ee372420) != nullptr when other.length() > 0 

### inline [LoanableSequence](#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE) &operator=(const [LoanableSequence](#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE) &other)

Copy the contents of another sequence into this one.

This method performs a deep copy of the sequence received into this one. If this sequence had a buffer loaned, it will behave as if [unloan](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a52f1963fb662a8e3a8a07a9757dd7aca) has been called. Allocations will happen when (a) [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == false and other.length() > 0 (b) [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true and other.length() > [maximum()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97)

* **Parameters:**
  **other** – **[in]** The sequence from where contents are to be copied.
* **Post:**
  [has_ownership()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true 
* **Post:**
  [maximum()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) >= other.length() 
* **Post:**
  [length()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == other.length() 
* **Post:**
  [buffer()](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1ad025ab06796dabef67b51be8ee372420) != nullptr when other.length() > 0 

### FASTDDS_SEQUENCE(FooSeq, Foo) using FooSeq = [eprosima::fastdds::dds::LoanableSequence](#classeprosima_1_1fastdds_1_1dds_1_1LoanableSequence)<Foo>
