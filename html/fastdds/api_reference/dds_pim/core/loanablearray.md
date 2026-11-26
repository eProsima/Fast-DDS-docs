<a id="api-pim-loanablearray"></a>

<a id="loanablearray"></a>

# LoanableArray

### template<typename T, std::size_t num_items>struct LoanableArray : public std::array<[T](#_CPPv4I0_NSt6size_tEEN8eprosima7fastdds3dds13LoanableArrayE), [num_items](#_CPPv4I0_NSt6size_tEEN8eprosima7fastdds3dds13LoanableArrayE)>

A type-safe, ordered collection of elements allocated on the stack, which can be loaned to a [LoanableCollection](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection). 

### Public Functions

### inline void \*\*buffer_for_loans() const

Get a buffer pointer that could be used on [LoanableCollection::loan](loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a5616e58617067b633242bbfe130d6d21).

* **Returns:**
  buffer pointer for loans.
