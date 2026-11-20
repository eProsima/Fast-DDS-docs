<a id="rtps-layer-time-t"></a>

# RTPS layer Time_t

### class Time_t

Structure [Time_t](#classeprosima_1_1fastdds_1_1rtps_1_1Time__t), used to describe times at RTPS protocol. 

### Public Functions

### Time_t() = default

Default constructor. Sets values to zero. 

### Time_t(std::int32_t sec, std::uint32_t frac)

* **Parameters:**
  * **sec** – Seconds 
  * **frac** – Fraction of second 

### Time_t(long double sec)

* **Parameters:**
  **sec** – Seconds. The fractional part is converted to nanoseconds. 

### Time_t(const eprosima::fastdds::dds::[Time_t](../../../dds_pim/core/Time_t/Time_t.md#_CPPv4N8eprosima7fastdds3dds6Time_tE) &time)

* **Parameters:**
  **time** – [fastdds::dds::Time_t](../../../dds_pim/core/Time_t/Time_t.md#structeprosima_1_1fastdds_1_1dds_1_1Time__t), aka. [dds::Duration_t](../../../dds_pim/core/Time_t/Duration_t.md#namespaceeprosima_1_1fastdds_1_1dds_1a03e7c84abe5e78952f6efb4067df6832). 

### std::int64_t to_ns() const

Returns stored time as nanoseconds (including seconds) 

### void from_ns(std::int64_t nanosecs)

* **Parameters:**
  **nanosecs** – Stores given time as nanoseconds (including seconds) 

### std::int32_t seconds() const

Retrieve the seconds field. 

### std::int32_t &seconds()

Retrieve the seconds field by ref. 

### void seconds(std::int32_t sec)

Sets seconds field. 

### std::uint32_t nanosec() const

Retrieve the nanosec field. 

### void nanosec(std::uint32_t nanos)

Sets nanoseconds field and updates the fraction. 

### std::uint32_t fraction() const

Retrieve the fraction field. 

### std::uint32_t &fraction()

Retrieve the fraction field by ref. 

### void fraction(std::uint32_t frac)

Sets fraction field and updates the nanoseconds. 

### Public Static Functions

### static void now([Time_t](#_CPPv4N8eprosima7fastdds4rtps6Time_tE) &ret)

Fills a [Time_t](#classeprosima_1_1fastdds_1_1rtps_1_1Time__t) struct with a representation of the current time.

* **Parameters:**
  **ret** – Reference to the structure to be filled in.
