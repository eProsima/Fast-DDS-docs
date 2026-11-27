<a id="python-api-pim-entity"></a>

<a id="entity"></a>

# Entity

### *class* fastdds.Entity(\*args, \*\*kwargs)

The Entity class is the abstract base class for all the objects that support QoS policies, a listener and
a status condition.

#### close()

This operation disables the Entity before closing it

#### enable()

This operation enables the Entity

* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### get_instance_handle()

Retrieves the instance handler that represents the Entity

* **Return type:**
  `InstanceHandle_t`
* **Returns:**
  Reference to the InstanceHandle

#### get_status_changes()

Retrieves the set of triggered statuses in the Entity

Triggered statuses are the ones whose value has changed
since the last time the application read the status.
When the entity is first created or if the entity is not enabled,
all communication statuses are in the non-triggered state,
so the list returned by the get_status_changes operation will be empty.
The list of statuses returned by the get_status_changes operation
refers to the status that are triggered on the Entity itself
and does not include statuses that apply to contained entities.

* **Return type:**
  [`StatusMask`](status/statusmask.md#fastdds.StatusMask)
* **Returns:**
  const reference to the StatusMask with the triggered statuses set to 1

#### get_status_mask()

Retrieves the set of relevant statuses for the Entity

* **Return type:**
  [`StatusMask`](status/statusmask.md#fastdds.StatusMask)
* **Returns:**
  Reference to the StatusMask with the relevant statuses set to 1

#### get_statuscondition(\*args)

*Overload 1:*

Allows access to the StatusCondition associated with the Entity

* **Return type:**
  `StatusCondition`
* **Returns:**
  Reference to StatusCondition object

<br/>

*Overload 2:*

Allows access to the StatusCondition associated with the Entity

* **Return type:**
  `StatusCondition`
* **Returns:**
  Const Reference to StatusCondition object

#### is_enabled()

Checks if the Entity is enabled

* **Return type:**
  boolean
* **Returns:**
  true if enabled, false if not

#### *property* thisown

The membership flag
