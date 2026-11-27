<a id="python-api-pim-durabilityqospolicy"></a>

<a id="durabilityqospolicy"></a>

# DurabilityQosPolicy

### *class* fastdds.DurabilityQosPolicy

This policy expresses if the data should ‘outlive’ their writing time.

Notes: Immutable Qos Policy

#### clear()

Clears the QosPolicy object

#### durabilityKind(\*args)

*Overload 1:*

Translates kind to rtps layer equivalent

* **Return type:**
  int
* **Returns:**
  fastdds::rtps::DurabilityKind_t

<br/>

*Overload 2:*

Set kind passing the rtps layer equivalent kind

* **Parameters:**
  **new_kind** (*int*) – fastdds::rtps::DurabilityKind_t

#### *property* kind

DurabilityQosPolicyKind.

By default the value for DataReaders: VOLATILE_DURABILITY_QOS, for DataWriters TRANSIENT_LOCAL_DURABILITY_QOS

#### *property* thisown

The membership flag
