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
  This happens, for instance, when the data type of a remote endpoint is not consistent with the local one under the
  [TypeConsistencyEnforcementQosPolicy](../../core/policy/xtypesExtensions.md#typeconsistencyenforcementqospolicy).
  See [InconsistentTopicStatus](../../core/status/status.md#dds-layer-core-status-inconsistenttopicstatus).

#### NOTE
**[Pro]** [`on_inconsistent_topic()`](../../../api_reference/dds_pim/topic/topiclistener.md#_CPPv4N8eprosima7fastdds3dds13TopicListener21on_inconsistent_topicEP5Topic23InconsistentTopicStatus) is only called in *Fast DDS* **[Pro]**.
In *Fast DDS* it is not implemented, so it is never called.

#### NOTE
**[Pro]** The library calls the overload that takes the [`InconsistentTopicStatus`](../../../api_reference/dds_pim/core/status/inconsistenttopicstatus.md#_CPPv4N8eprosima7fastdds3dds23InconsistentTopicStatusE) by const reference.
The overload that takes it by value is deprecated and will be removed in version 4.
The default implementation of the const reference overload calls the by-value one, so listeners that override
the deprecated overload keep receiving the callback.

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

    using TopicListener::on_inconsistent_topic;

    void on_inconsistent_topic(
            Topic* topic,
            const InconsistentTopicStatus& status) override
    {
        static_cast<void>(topic);
        static_cast<void>(status);
        std::cout << "Inconsistent topic received discovered" << std::endl;
    }

};
```
