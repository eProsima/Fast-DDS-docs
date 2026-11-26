<a id="python-api-pim-topiclistener"></a>

<a id="topiclistener"></a>

# TopicListener

### *class* fastdds.TopicListener

Class TopicListener, it should be used by the end user to implement specific callbacks to certain actions.

#### on_inconsistent_topic(topic, status)

\_pro Virtual function to be implemented by the user containing the actions to be performed when
another topic exists with the same name but different characteristics.

The default implementation calls the deprecated on_inconsistent_topic(Topic\*, InconsistentTopicStatus), so
listeners overriding it are still notified.

Notes: Both overloads are equally good for a call with an InconsistentTopicStatus argument, so a direct call
through a TopicListener is ambiguous. Call it through a pointer to this member function.

* **Parameters:**
  * **topic** ([`Topic`](topic_class.md#fastdds.Topic)) – Topic
  * **status** (`InconsistentTopicStatus`) – The inconsistent topic status

#### *property* thisown

The membership flag
