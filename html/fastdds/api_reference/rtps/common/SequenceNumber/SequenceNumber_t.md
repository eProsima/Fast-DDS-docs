<a id="sequencenumber-t"></a>

# SequenceNumber_t

### struct SequenceNumber_t

Structure [SequenceNumber_t](#structeprosima_1_1fastdds_1_1rtps_1_1SequenceNumber__t), different for each change in the same writer. 

### Public Functions

### inline SequenceNumber_t() noexcept

Default constructor. 

### inline SequenceNumber_t(std::int32_t hi, std::uint32_t lo) noexcept

* **Parameters:**
  * **hi** – 
  * **lo** – 

### inline explicit SequenceNumber_t(std::uint64_t u) noexcept

* **Parameters:**
  **u** – 

### inline std::uint64_t to64long() const noexcept

Convert the number to 64 bit. 

* **Returns:**
  64 bit representation of the SequenceNumber 

### inline [SequenceNumber_t](#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &operator++() noexcept

Increase SequenceNumber in 1. 

### inline [SequenceNumber_t](#_CPPv4N8eprosima7fastdds4rtps16SequenceNumber_tE) &operator+=(int inc) noexcept

Increase SequenceNumber. 

* **Parameters:**
  **inc** – Number to add to the SequenceNumber
