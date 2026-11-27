<a id="python-api-pim-typesupport"></a>

<a id="typesupport"></a>

# TypeSupport

### *class* fastdds.TypeSupport(\*args)

Notes: This class inherits from std::shared_ptr<TopicDataType>.
Class TypeSupport used to provide the DomainRTPSParticipant with the methods to serialize,
deserialize and get the key of a specific data type.
The user should created a class that inherits from this one,
where Serialize and deserialize methods MUST be implemented.

#### calculate_serialized_size(data, data_representation)

Returns a function which can be used to calculate the serialized size of the provided data.

* **Parameters:**
  * **[****in****]** – data Pointer to data.
  * **[****in****]** – data_representation Representation that should be used for calculating the serialized size.
* **Return type:**
  int
* **Returns:**
  Functor which calculates the serialized size of the data.

#### compute_key(\*args)

*Overload 1:*

Getter for the data key

* **Parameters:**
  * **data** (*void*) – Pointer to serialized payload containing the data.
  * **i_handle** (`InstanceHandle_t`) – InstanceHandle pointer to store the key
  * **force_md5** (*boolean* *,* *optional*) – boolean to force md5 (default: false)
* **Return type:**
  boolean
* **Returns:**
  true if the key is returned, false if not

<br/>

*Overload 2:*

Getter for the data key

* **Parameters:**
  * **payload** (*eprosima::fastdds::rtps::SerializedPayload_t*) – Pointer to data
  * **i_handle** (`InstanceHandle_t`) – InstanceHandle pointer to store the key
  * **force_md5** (*boolean* *,* *optional*) – boolean to force md5 (default: false)
* **Return type:**
  boolean
* **Returns:**
  true if the key is returned, false if not

<br/>

*Overload 3:*

Getter for the data key

* **Parameters:**
  * **payload** (*eprosima::fastdds::rtps::SerializedPayload_t*) – Pointer to data
  * **i_handle** (`InstanceHandle_t`) – InstanceHandle pointer to store the key
  * **force_md5** – boolean to force md5 (default: false)
* **Return type:**
  boolean
* **Returns:**
  true if the key is returned, false if not

#### create_data()

Creates new data

* **Return type:**
  void
* **Returns:**
  Pointer to the data

#### delete_data(data)

Deletes data

* **Parameters:**
  **data** (*void*) – Pointer to the data to delete

#### deserialize(payload, data)

Deserializes the data

* **Parameters:**
  * **payload** (*eprosima::fastdds::rtps::SerializedPayload_t*) – Pointer to payload
  * **data** (*void*) – Pointer to data
* **Return type:**
  boolean
* **Returns:**
  true if it is deserialized correctly, false if not

#### empty()

Check if the TypeSupport is empty

* **Return type:**
  boolean
* **Returns:**
  true if empty, false if not

#### get_type_name()

Getter for the type name

* **Return type:**
  string
* **Returns:**
  name of the data type

#### is_bounded()

Checks if the type is bounded.

#### is_plain(data_representation)

Checks if the type is plain when using a specific encoding.

#### register_type(\*args)

*Overload 1:*

Registers the type on a participant

* **Parameters:**
  **participant** ([`DomainParticipant`](../domain/domainparticipant.md#fastdds.DomainParticipant)) – DomainParticipant where the type is going to be registered
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the type name is empty, RETCODE_PRECONDITION_NOT_MET if there is another type with
  the same name registered on the DomainParticipant and RETCODE_OK if it is registered correctly

<br/>

*Overload 2:*

Registers the type on a participant

* **Parameters:**
  * **participant** ([`DomainParticipant`](../domain/domainparticipant.md#fastdds.DomainParticipant)) – DomainParticipant where the type is going to be registered
  * **type_name** (*string*) – Name of the type to register
* **Return type:**
  int
* **Returns:**
  RETCODE_BAD_PARAMETER if the type name is empty, RETCODE_PRECONDITION_NOT_MET if there is another type with
  the same name registered on the DomainParticipant and RETCODE_OK if it is registered correctly

#### serialize(data, payload, data_representation)

Serializes the data

* **Parameters:**
  * **data** (*void*) – Pointer to data
  * **payload** (*eprosima::fastdds::rtps::SerializedPayload_t*) – Pointer to payload
  * **[****in****]** – data_representation Representation that should be used to encode the data into the payload.
* **Return type:**
  boolean
* **Returns:**
  true if it is serialized correctly, false if not

#### set(ptr)

#### *property* thisown

The membership flag
