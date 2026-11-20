<a id="python-api-pim-domainparticipantfactoryqos"></a>

<a id="domainparticipantfactoryqos"></a>

# DomainParticipantFactoryQos

### *class* fastdds.DomainParticipantFactoryQos

Class DomainParticipantFactoryQos, contains all the possible Qos that can be set for a determined participant.
Please consult each of them to check for implementation details and default values.

#### entity_factory(\*args)

*Overload 1:*

Getter for EntityFactoryQosPolicy
:rtype: [`EntityFactoryQosPolicy`](../core/policy/entityfactoryqospolicy.md#fastdds.EntityFactoryQosPolicy)
:return: EntityFactoryQosPolicy reference

<br/>

*Overload 2:*

Getter for EntityFactoryQosPolicy
:rtype: [`EntityFactoryQosPolicy`](../core/policy/entityfactoryqospolicy.md#fastdds.EntityFactoryQosPolicy)
:return: EntityFactoryQosPolicy reference

<br/>

*Overload 3:*

Setter for EntityFactoryQosPolicy
:type entity_factory: [`EntityFactoryQosPolicy`](../core/policy/entityfactoryqospolicy.md#fastdds.EntityFactoryQosPolicy)
:param entity_factory: EntityFactoryQosPolicy

#### file_watch_threads(\*args)

*Overload 1:*

Getter for file watch related ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 2:*

Getter for file watch related ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 3:*

Setter for the file watch related ThreadSettings

* **Parameters:**
  **value** (`ThreadSettings`) – New ThreadSettings to be set

#### shm_watchdog_thread(\*args)

*Overload 1:*

Getter for SHM watchdog ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 2:*

Getter for SHM watchdog ThreadSettings

* **Return type:**
  `ThreadSettings`
* **Returns:**
  rtps::ThreadSettings reference

<br/>

*Overload 3:*

Setter for the SHM watchdog ThreadSettings

* **Parameters:**
  **value** (`ThreadSettings`) – New ThreadSettings to be set

#### *property* thisown

The membership flag
