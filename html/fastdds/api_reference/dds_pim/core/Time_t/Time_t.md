<a id="api-pim-time-t"></a>

<a id="dds-layer-time-t"></a>

# DDS Layer Time_t

### struct Time_t

Structure [Time_t](#structeprosima_1_1fastdds_1_1dds_1_1Time__t), used to describe times at a DDS level. 

### Public Functions

### Time_t()

Default constructor. Sets values to zero. 

### Time_t(std::int32_t sec, std::uint32_t nsec)

* **Parameters:**
  * **sec** – Seconds 
  * **nsec** – Nanoseconds 

### Time_t(long double sec)

* **Parameters:**
  **sec** – Seconds. The fractional part is converted to nanoseconds. 

### std::int64_t to_ns() const

Returns stored time as nanoseconds (including seconds) 

### Public Static Functions

### static void now([Time_t](#_CPPv4N8eprosima7fastdds3dds6Time_tE) &ret)

Fills a [Time_t](#structeprosima_1_1fastdds_1_1dds_1_1Time__t) struct with a representation of the current time.

* **Parameters:**
  **ret** – Reference to the structure to be filled in.
