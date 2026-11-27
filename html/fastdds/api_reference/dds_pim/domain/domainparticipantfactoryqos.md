<a id="api-pim-domainparticipantfactoryqos"></a>

<a id="domainparticipantfactoryqos"></a>

# DomainParticipantFactoryQos

### class DomainParticipantFactoryQos

Class [DomainParticipantFactoryQos](#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipantFactoryQos), contains all the possible Qos that can be set for a determined participant. Please consult each of them to check for implementation details and default values. 

### Public Functions

### inline DomainParticipantFactoryQos()

Constructor. 

### inline virtual ~DomainParticipantFactoryQos()

Destructor. 

### inline const [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory() const

Getter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) 

* **Returns:**
  [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) reference 

### inline [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory()

Getter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) 

* **Returns:**
  [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) reference 

### inline void entity_factory(const [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#_CPPv4N8eprosima7fastdds3dds22EntityFactoryQosPolicyE) &entity_factory)

Setter for [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy) 

* **Parameters:**
  **entity_factory** – [EntityFactoryQosPolicy](../core/policy/entityfactoryqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1EntityFactoryQosPolicy)

### inline rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &shm_watchdog_thread()

Getter for SHM watchdog ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &shm_watchdog_thread() const

Getter for SHM watchdog ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline void shm_watchdog_thread(const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the SHM watchdog ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set 

### inline rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &file_watch_threads()

Getter for file watch related ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &file_watch_threads() const

Getter for file watch related ThreadSettings

* **Returns:**
  [rtps::ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) reference 

### inline void file_watch_threads(const rtps::[ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &value)

Setter for the file watch related ThreadSettings

* **Parameters:**
  **value** – New ThreadSettings to be set
