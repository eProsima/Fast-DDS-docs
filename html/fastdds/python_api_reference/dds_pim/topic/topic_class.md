<a id="python-api-pim-topic-class"></a>

<a id="topic"></a>

# Topic

### *class* fastdds.Topic(\*args, \*\*kwargs)

Class Topic, represents the fact that both publications
and subscriptions are tied to a single data-type

#### get_impl()

#### get_listener()

Retrieves the attached TopicListener.

* **Return type:**
  [`TopicListener`](topiclistener.md#fastdds.TopicListener)
* **Returns:**
  pointer to TopicListener

#### get_participant()

Getter for the DomainParticipant

* **Return type:**
  [`DomainParticipant`](../domain/domainparticipant.md#fastdds.DomainParticipant)
* **Returns:**
  DomainParticipant pointer

#### get_qos(\*args)

*Overload 1:*

Allows accessing the Topic Qos.

* **Return type:**
  [`TopicQos`](topicqos.md#fastdds.TopicQos)
* **Returns:**
  reference to TopicQos

<br/>

*Overload 2:*

Retrieves the Topic Qos.

* **Parameters:**
  **qos** ([`TopicQos`](topicqos.md#fastdds.TopicQos)) – TopicQos where the qos is returned
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### set_listener(\*args)

Modifies the TopicListener.

* **Parameters:**
  * **listener** ([`TopicListener`](topiclistener.md#fastdds.TopicListener)) – new value for the TopicListener
  * **mask** ([`StatusMask`](../core/status/statusmask.md#fastdds.StatusMask), optional) – StatusMask that holds statuses the listener responds to (default: all).
* **Return type:**
  int
* **Returns:**
  RETCODE_OK

#### set_qos(qos)

Allows modifying the Topic Qos.
The given Qos must be supported by the Topic.

* **Parameters:**
  **qos** ([`TopicQos`](topicqos.md#fastdds.TopicQos)) – new TopicQos value to set for the Topic.

#### *property* thisown

The membership flag
