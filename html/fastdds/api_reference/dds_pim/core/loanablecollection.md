<a id="api-pim-loanablecollection"></a>

<a id="loanablecollection"></a>

# LoanableCollection

### class LoanableCollection

A collection of generic opaque pointers that can receive the buffer from outside (loan).

This is an abstract class. See [LoanableSequence](loanablesequence.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableSequence) for details. 

Subclassed by eprosima::fastdds::dds::LoanableTypedCollection< T, std::true_type >, eprosima::fastdds::dds::LoanableTypedCollection< T, \_NonConstEnabler >, eprosima::fastdds::dds::UserAllocatedSequence

### Public Functions

### inline const element_type \*buffer() const

Get the pointer to the elements buffer.

The returned value may be nullptr if [maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) is 0. Otherwise it is guaranteed that up to [maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) elements can be accessed.

* **Returns:**
  the pointer to the elements buffer. 

### inline bool has_ownership() const

Get the ownership flag.

* **Returns:**
  whether the collection has ownership of the buffer. 

### inline size_type maximum() const

Get the maximum number of elements currently allocated.

* **Returns:**
  the maximum number of elements currently allocated. 

### inline size_type length() const

Get the number of elements currently accessible.

* **Returns:**
  the number of elements currently accessible. 

### inline bool length(size_type new_length)

Set the number of elements currently accessible.

This method tells the collection that a certain number of elements should be accessible. If the new length is greater than the current [maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) the collection should allocate space for the new elements. If this is the case and the collection does not own the buffer (i.e. [has_ownership()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) is false) then no allocation will be performed, the length will remain unchanged, and false will be returned.

* **Parameters:**
  **new_length** – **[in]** New number of elements to be accessible.
* **Pre:**
  new_length >= 0
* **Returns:**
  true if the new length was correctly set.
* **Post:**
  [length()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == new_length 
* **Post:**
  [maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) >= new_length 

### inline bool loan(element_type \*buffer, size_type new_maximum, size_type new_length)

Loan a buffer to the collection.

* **Parameters:**
  * **buffer** – **[in]** pointer to the buffer to be loaned. 
  * **new_maximum** – **[in]** number of allocated elements in buffer. 
  * **new_length** – **[in]** number of accessible elements in buffer.
* **Pre:**
  ([has_ownership()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == false) || ([maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == 0) 
* **Pre:**
  new_maximum > 0 
* **Pre:**
  new_maximum >= new_length 
* **Pre:**
  buffer != nullptr
* **Returns:**
  false if preconditions are not met. 
* **Returns:**
  true if operation succeeds.
* **Post:**
  [buffer()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1ad025ab06796dabef67b51be8ee372420) == buffer 
* **Post:**
  [has_ownership()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == false 
* **Post:**
  [maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == new_maximum 
* **Post:**
  [length()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == new_length 

### inline element_type \*unloan(size_type &maximum, size_type &length)

Remove the loan from the collection.

* **Parameters:**
  * **maximum** – **[out]** number of allocated elements on the returned buffer. 
  * **length** – **[out]** number of accessible elements on the returned buffer.
* **Pre:**
  [has_ownership()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == false
* **Returns:**
  nullptr if preconditions are not met. 
* **Returns:**
  pointer to the previously loaned buffer of elements.
* **Post:**
  [buffer()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1ad025ab06796dabef67b51be8ee372420) == nullptr 
* **Post:**
  [has_ownership()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true 
* **Post:**
  [length()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == 0 
* **Post:**
  [maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == 0 

### inline element_type \*unloan()

Remove the loan from the collection.

* **Pre:**
  [has_ownership()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == false
* **Returns:**
  nullptr if preconditions are not met. 
* **Returns:**
  pointer to the previously loaned buffer of elements.
* **Post:**
  [buffer()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1ad025ab06796dabef67b51be8ee372420) == nullptr 
* **Post:**
  [has_ownership()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1af14322e027dc657b96a87fceb0f1c05a) == true 
* **Post:**
  [length()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a7b4799b9dc973f67e34148ba4fefbfac) == 0 
* **Post:**
  [maximum()](#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection_1a3a7bed2e6b0efb105e280fc5d4c44c97) == 0
