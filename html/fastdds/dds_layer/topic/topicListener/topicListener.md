<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-topiclistener"></a>

<a id="topiclistener"></a>

# TopicListener

[`TopicListener`](../../../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListenerE) is an abstract class defining the callbacks that will be triggered
in response to state changes on the [Topic](../topic/topic.md#dds-layer-topic-topic).
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

TopicListener has the following callback:

* [`on_inconsistent_topic()`](../../../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListener21on_inconsistent_topicEP5Topic23InconsistentTopicStatus): A remote Topic is discovered with the same name
  but different characteristics as another locally created Topic.

#### NOTE
**[Pro]** [`on_inconsistent_topic()`](../../../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListener21on_inconsistent_topicEP5Topic23InconsistentTopicStatus) is only called in *Fast DDS* **[Pro]**.
In *Fast DDS* it is not implemented, so it is never called.

```c++
class CustomTopicListener : public TopicListener
{

public:

    CustomTopicListener()
        : TopicListener()
    {
    }

    virtual ~CustomTopicListener()
    {
    }

    void on_inconsistent_topic(
            Topic* topic,
            InconsistentTopicStatus status) override
    {
        static_cast<void>(topic);
        static_cast<void>(status);
        std::cout << "Inconsistent topic received discovered" << std::endl;
    }

};
```
