<a id="writeparams"></a>

# WriteParams

### class WriteParams

This class contains additional information of a CacheChange. 

### Public Functions

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &sample_identity(const [SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &sample_id)

Set the value of the sample_identity member.

* **Parameters:**
  **sample_id** – New value for the sample_identity member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &sample_identity([SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &&sample_id)

Set the value of the sample_identity member.

* **Parameters:**
  **sample_id** – New value for the sample_identity member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### inline const [SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &sample_identity() const

Get the value of the sample_identity member.

* **Returns:**
  Constant reference to the sample_identity member. 

### inline [SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &sample_identity()

Set the value of the sample_identity member.

* **Returns:**
  Reference to the sample_identity member. 

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &related_sample_identity(const [SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &sample_id)

Set the value of the related_sample_identity member of this class.

* **Parameters:**
  **sample_id** – New value for the related_sample_identity member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &related_sample_identity([SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &&sample_id)

Set the related_sample_identity member of this class.

* **Parameters:**
  **sample_id** – New value for the related_sample_identity member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### inline const [SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &related_sample_identity() const

Get the value of the related_sample_identity member.

* **Returns:**
  Constant reference to the related_sample_identity member. 

### inline [SampleIdentity](../SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) &related_sample_identity()

Set the value of the related_sample_identity member.

* **Returns:**
  Reference to the related_sample_identity member. 

### inline [Time_t](../Time_t/Time_t.md#_CPPv4N8eprosima7fastdds4rtps6Time_tE) source_timestamp() const

Get the value of the source_timestamp member.

* **Returns:**
  Current value of the source_timestamp member. 

### inline [Time_t](../Time_t/Time_t.md#_CPPv4N8eprosima7fastdds4rtps6Time_tE) &source_timestamp()

Set the value of the source_timestamp member.

* **Returns:**
  Reference to the source_timestamp member. 

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &source_timestamp(const [Time_t](../Time_t/Time_t.md#_CPPv4N8eprosima7fastdds4rtps6Time_tE) &timestamp)

Set the source_timestamp member of this class.

* **Parameters:**
  **timestamp** – New value for the source_timestamp member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &source_timestamp([Time_t](../Time_t/Time_t.md#_CPPv4N8eprosima7fastdds4rtps6Time_tE) &&timestamp)

Set the source_timestamp member of this class.

* **Parameters:**
  **timestamp** – New value for the source_timestamp member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### inline std::shared_ptr<[UserWriteData](#_CPPv4N8eprosima7fastdds4rtps11WriteParams13UserWriteDataE)> user_write_data() const

Retrieves the user write data. 

* **Returns:**
  Shared pointer to the user write data. 

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &user_write_data(std::shared_ptr<[UserWriteData](#_CPPv4N8eprosima7fastdds4rtps11WriteParams13UserWriteDataE)> write_data)

Set the user write data.

* **Parameters:**
  **write_data** – New value for the user_write_data member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### inline const OriginalWriterInfo &original_writer_info() const

Get the Original Writer Info. 

* **Returns:**
  Constant reference to the original_writer_info_ member. 

### inline OriginalWriterInfo &original_writer_info()

Get the Original Writer Info. 

* **Returns:**
  Constant reference to the original_writer_info_ member. 

### inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) &original_writer_info(const OriginalWriterInfo &original_writer)

Set the Original Writer Info of the original writer. 

* **Parameters:**
  **original_writer** – New value for the original_writer_info_ member.
* **Returns:**
  Reference to the modified object in order to allow daisy chaining. 

### Public Static Functions

### static inline [WriteParams](#_CPPv4N8eprosima7fastdds4rtps11WriteParamsE) write_params_default() noexcept

Default value for methods receiving a [WriteParams](#classeprosima_1_1fastdds_1_1rtps_1_1WriteParams).

Will contain the following values on its members:
* sample_identity: Invalid [SampleIdentity](../SampleIdentity/SampleIdentity.md#classeprosima_1_1fastdds_1_1rtps_1_1SampleIdentity)
* related_sample_identity: Invalid [SampleIdentity](../SampleIdentity/SampleIdentity.md#classeprosima_1_1fastdds_1_1rtps_1_1SampleIdentity)
* source_timestamp: Invalid [Time_t](../Time_t/Time_t.md#classeprosima_1_1fastdds_1_1rtps_1_1Time__t)

#### NOTE
This should not return a reference to the static value if this value is meant to be read and written from different threads. 

### struct UserWriteData

Base class storing custom information in the [WriteParams](#classeprosima_1_1fastdds_1_1rtps_1_1WriteParams) structure for later filtering. 

This struct serves as a base class that allows derived classes to be deleted safely through a pointer to this base type. It is intended to be user-extensible.
