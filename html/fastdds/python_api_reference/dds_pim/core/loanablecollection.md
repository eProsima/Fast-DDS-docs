<a id="python-api-pim-loanablecollection"></a>

<a id="loanablecollection"></a>

# LoanableCollection

### *class* fastdds.LoanableCollection(\*args, \*\*kwargs)

A collection of generic opaque pointers that can receive the buffer from outside (loan).

This is an abstract class. See ‘LoanableSequence’ for details.

#### buffer()

Get the pointer to the elements buffer.

The returned value may be nullptr if maximum() is 0.
Otherwise it is guaranteed that up to maximum() elements can be accessed.

* **Return type:**
  void
* **Returns:**
  the pointer to the elements buffer.

#### has_ownership()

Get the ownership flag.

* **Return type:**
  boolean
* **Returns:**
  whether the collection has ownership of the buffer.

#### length(\*args)

*Overload 1:*

Get the number of elements currently accessible.

* **Return type:**
  int
* **Returns:**
  the number of elements currently accessible.

<br/>

*Overload 2:*

Set the number of elements currently accessible.

This method tells the collection that a certain number of elements should be accessible.
If the new length is greater than the current ‘maximum()’ the collection should allocate
space for the new elements. If this is the case and the collection does not own the buffer
(i.e. ‘has_ownership()’ is false) then no allocation will be performed, the length will
remain unchanged, and false will be returned.

* **Parameters:**
  **[****in****]** – new_length New number of elements to be accessible.
* **Return type:**
  boolean
* **Returns:**
  true if the new length was correctly set.

#### loan(buffer, new_maximum, new_length)

Loan a buffer to the collection.

* **Parameters:**
  * **[****in****]** – buffer       pointer to the buffer to be loaned.
  * **[****in****]** – new_maximum  number of allocated elements in buffer.
  * **[****in****]** – new_length   number of accessible elements in buffer.
* **Return type:**
  boolean
* **Returns:**
  false if preconditions are not met.
* **Return type:**
  boolean
* **Returns:**
  true if operation succeeds.

#### maximum()

Get the maximum number of elements currently allocated.

* **Return type:**
  int
* **Returns:**
  the maximum number of elements currently allocated.

#### *property* thisown

The membership flag

#### unloan(\*args)

*Overload 1:*

Remove the loan from the collection.

* **Parameters:**
  * **[****out****]** – maximum  number of allocated elements on the returned buffer.
  * **[****out****]** – length   number of accessible elements on the returned buffer.
* **Return type:**
  void
* **Returns:**
  nullptr if preconditions are not met.
* **Return type:**
  void
* **Returns:**
  pointer to the previously loaned buffer of elements.

<br/>

*Overload 2:*

Remove the loan from the collection.

* **Return type:**
  void
* **Returns:**
  nullptr if preconditions are not met.
* **Return type:**
  void
* **Returns:**
  pointer to the previously loaned buffer of elements.
