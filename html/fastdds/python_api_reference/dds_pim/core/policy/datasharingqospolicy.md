<a id="python-api-pim-datasharingqospolicy"></a>

<a id="datasharingqospolicy"></a>

# DataSharingQosPolicy

### *class* fastdds.DataSharingQosPolicy(\*args)

Qos Policy to configure the data sharing

Notes: Immutable Qos Policy

#### add_domain_id(\*args)

#### automatic(\*args)

*Overload 1:*

Configures the DataSharing in automatic mode

The default shared memory directory of the OS is used.
A default domain ID is automatically computed.

<br/>

*Overload 2:*

Configures the DataSharing in automatic mode

The default shared memory directory of the OS is used.

* **Parameters:**
  **domain_ids** (*std::vector< std::uint16_t* *,**std::allocator< std::uint16_t > >*) – the user configured DataSharing domain IDs (16 bits).

<br/>

*Overload 3:*

Configures the DataSharing in automatic mode

A default domain ID is automatically computed.

* **Parameters:**
  **directory** (*string*) – The shared memory directory to use.

<br/>

*Overload 4:*

Configures the DataSharing in automatic mode

* **Parameters:**
  * **directory** (*string*) – The shared memory directory to use.
  * **domain_ids** (*std::vector< std::uint16_t* *,**std::allocator< std::uint16_t > >*) – the user configured DataSharing domain IDs (16 bits).

#### clear()

Clears the QosPolicy object

#### data_sharing_listener_thread(\*args)

*Overload 1:*

Getter for DataSharing listener thread ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 2:*

Getter for DataSharing listener thread ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 3:*

Setter for the DataSharing listener thread ThreadSettings

* **Parameters:**
  **value** (`ThreadSettings`) – New ThreadSettings to be set

#### domain_ids()

Gets the set of DataSharing domain IDs.

Each domain ID is 64 bit long.
However, user-defined domain IDs are only 16 bit long,
while the rest of the 48 bits are used for the
automatically generated domain ID (if any).

- Automatic domain IDs use the 48 MSB and leave the 16 LSB as zero.
- User defined domain IDs use the 16 LSB and leave the 48 MSB as zero.

* **Return type:**
  std::vector< std::uint64_t,std::allocator< std::uint64_t > >
* **Returns:**
  the current DataSharing domain IDs

#### kind()

* **Return type:**
  int
* **Returns:**
  the current DataSharing configuration mode

#### max_domains()

* **Return type:**
  int
* **Returns:**
  the current configured maximum number of domain IDs

#### off()

Configures the DataSharing in disabled mode

#### on(\*args)

*Overload 1:*

Configures the DataSharing in active mode

A default domain ID is automatically computed.

* **Parameters:**
  **directory** (*string*) – The shared memory directory to use.
  It is mandatory to provide a non-empty name or the creation of endpoints will fail.

<br/>

*Overload 2:*

Configures the DataSharing in active mode

* **Parameters:**
  * **directory** (*string*) – The shared memory directory to use.
    It is mandatory to provide a non-empty name or the creation of endpoints will fail.
  * **domain_ids** (*std::vector< std::uint16_t* *,**std::allocator< std::uint16_t > >*) – the user configured DataSharing domain IDs (16 bits).

#### set_max_domains(size)

* **Parameters:**
  **size** (*int*) – the new maximum number of domain IDs

#### shm_directory()

* **Return type:**
  string
* **Returns:**
  the current DataSharing shared memory directory

#### *property* thisown

The membership flag
