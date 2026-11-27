<a id="locator-t"></a>

# Locator_t

### class Locator_t

Class [Locator_t](#classeprosima_1_1fastdds_1_1rtps_1_1Locator__t), uniquely identifies a communication channel for a particular transport. For example, an address + port combination in the case of UDP. 

Subclassed by [eprosima::fastdds::rtps::LocatorWithMask](../LocatorWithMask/LocatorWithMask.md#classeprosima_1_1fastdds_1_1rtps_1_1LocatorWithMask)

### Public Functions

### inline Locator_t()

Default constructor. 

### inline Locator_t([Locator_t](#_CPPv4N8eprosima7fastdds4rtps9Locator_t9Locator_tERR9Locator_t) &&loc)

Move constructor. 

### inline Locator_t(const [Locator_t](#_CPPv4N8eprosima7fastdds4rtps9Locator_t9Locator_tERK9Locator_t) &loc)

Copy constructor. 

### inline Locator_t(std::uint32_t portin)

Port constructor. 

### inline Locator_t(std::int32_t kindin, std::uint32_t portin)

Kind and port constructor. 

### inline [Locator_t](#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &operator=(const [Locator_t](#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc)

Copy assignment. 

### inline bool set_address(const [Locator_t](#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &other)

Set the locator IP address using another locator. 

* **Parameters:**
  **other** – Locator which IP address is used to set this locator IP address. 
* **Returns:**
  always true. 

### inline [octet](../Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) \*get_address()

Getter for the locator IP address. 

* **Returns:**
  IP address as octet pointer. 

### inline [octet](../Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) get_address(std::uint16_t field) const

Getter for a specific field of the locator IP address. 

* **Parameters:**
  **field** – IP address element to be accessed. 
* **Returns:**
  Octet value for the specific IP address element. 

### inline void set_Invalid_Address()

Automatic setter for setting locator IP address to invalid address (0). 

### Public Members

### std::int32_t kind

Specifies the locator type. Valid values are: 

LOCATOR_KIND_UDPv4

LOCATOR_KIND_UDPv6

LOCATOR_KIND_TCPv4

LOCATOR_KIND_TCPv6

LOCATOR_KIND_SHM

LOCATOR_KIND_ETHERNET 

### std::uint32_t port

Network port. 

### [octet](../Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) address[16]

IP address. 

### Public Static Functions

### static [Locator_t](#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) create_locator(std::int32_t kind, const std::string &address, std::uint32_t port)

Create a locator with the given parameters. 

* **Parameters:**
  * **kind** – Kind of the locator. 
  * **address** – IP Address of the locator as string. 
  * **port** – Port of the locator. 
* **Returns:**
  [Locator_t](#classeprosima_1_1fastdds_1_1rtps_1_1Locator__t) object initialized with the given parameters.
