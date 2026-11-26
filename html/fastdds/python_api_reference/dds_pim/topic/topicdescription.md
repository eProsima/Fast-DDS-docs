<a id="python-api-pim-topicdescription"></a>

<a id="topicdescription"></a>

# TopicDescription

### *class* fastdds.TopicDescription(\*args, \*\*kwargs)

Class TopicDescription, represents the fact that both publications
and subscriptions are tied to a single data-type

#### get_impl()

#### get_name()

Get the name used to create this TopicDescription.

* **Return type:**
  string
* **Returns:**
  the name used to create this TopicDescription.

#### get_participant()

Get the DomainParticipant to which the TopicDescription belongs.

* **Return type:**
  [`DomainParticipant`](../domain/domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  The DomainParticipant to which the TopicDescription belongs.

#### get_type_name()

Get the associated type name.

* **Return type:**
  string
* **Returns:**
  the type name.

#### *property* thisown

The membership flag
