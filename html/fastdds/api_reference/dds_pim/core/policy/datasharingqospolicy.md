<a id="api-pim-datasharingqospolicy"></a>

<a id="datasharingqospolicy"></a>

# DataSharingQosPolicy

### class DataSharingQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Qos Policy to configure the data sharing

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline DataSharingQosPolicy()

Constructor. 

### virtual ~DataSharingQosPolicy() = default

Destructor. 

### inline DataSharingQosPolicy(const [DataSharingQosPolicy](#_CPPv4N8eprosima7fastdds3dds20DataSharingQosPolicy20DataSharingQosPolicyERK20DataSharingQosPolicy) &b)

Copy constructor. 

* **Parameters:**
  **b** – Another [DataSharingQosPolicy](#classeprosima_1_1fastdds_1_1dds_1_1DataSharingQosPolicy) instance 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### inline const [DataSharingKind](datasharingkind.md#_CPPv4N8eprosima7fastdds3dds15DataSharingKindE) &kind() const

* **Returns:**
  the current DataSharing configuration mode 

### inline const std::string &shm_directory() const

* **Returns:**
  the current DataSharing shared memory directory 

### inline const std::vector<std::uint64_t> &domain_ids() const

Gets the set of DataSharing domain IDs.

Each domain ID is 64 bit long. However, user-defined domain IDs are only 16 bit long, while the rest of the 48 bits are used for the automatically generated domain ID (if any).

* Automatic domain IDs use the 48 MSB and leave the 16 LSB as zero.
* User defined domain IDs use the 16 LSB and leave the 48 MSB as zero.

* **Returns:**
  the current DataSharing domain IDs 

### inline void set_max_domains(std::uint32_t size)

* **Parameters:**
  **size** – the new maximum number of domain IDs 

### inline const std::uint32_t &max_domains() const

* **Returns:**
  the current configured maximum number of domain IDs 

### inline void automatic()

Configures the DataSharing in automatic mode. 

The default shared memory directory of the OS is used. A default domain ID is automatically computed. 

### inline void automatic(const std::vector<std::uint16_t> &domain_ids)

Configures the DataSharing in automatic mode. 

The default shared memory directory of the OS is used.

* **Parameters:**
  **domain_ids** – the user configured DataSharing domain IDs (16 bits). 

### inline void automatic(const std::string &directory)

Configures the DataSharing in automatic mode. 

A default domain ID is automatically computed.

* **Parameters:**
  **directory** – The shared memory directory to use. 

### inline void automatic(const std::string &directory, const std::vector<std::uint16_t> &domain_ids)

Configures the DataSharing in automatic mode. 

* **Parameters:**
  * **directory** – The shared memory directory to use. 
  * **domain_ids** – the user configured DataSharing domain IDs (16 bits). 

### inline void on(const std::string &directory)

Configures the DataSharing in active mode. 

A default domain ID is automatically computed.

* **Parameters:**
  **directory** – The shared memory directory to use. It is mandatory to provide a non-empty name or the creation of endpoints will fail. 

### inline void on(const std::string &directory, const std::vector<std::uint16_t> &domain_ids)

Configures the DataSharing in active mode. 

* **Parameters:**
  * **directory** – The shared memory directory to use. It is mandatory to provide a non-empty name or the creation of endpoints will fail. 
  * **domain_ids** – the user configured DataSharing domain IDs (16 bits). 

### inline void off()

Configures the DataSharing in disabled mode. 

### inline void add_domain_id(std::uint16_t id)

Adds a user-specific DataSharing domain ID. 

* **Parameters:**
  **id** – 16 bit identifier 

### inline rtps::[ThreadSettings](../../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &data_sharing_listener_thread()

Getter for DataSharing listener thread ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &data_sharing_listener_thread() const

Getter for DataSharing listener thread ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline void data_sharing_listener_thread(const rtps::[ThreadSettings](../../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the DataSharing listener thread ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set
