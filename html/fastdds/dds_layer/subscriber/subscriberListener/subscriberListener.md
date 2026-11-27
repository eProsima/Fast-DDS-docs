<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-subscriberlistener"></a>

<a id="subscriberlistener"></a>

# SubscriberListener

[`SubscriberListener`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListenerE) is an abstract class defining the
callbacks that will be triggered in response to state changes on the
[Subscriber](../subscriber/subscriber.md#dds-layer-subscriber-subscriber).
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

SubscriberListener inherits from [DataReaderListener](../dataReaderListener/dataReaderListener.md#dds-layer-subscriber-datareaderlistener).
Therefore, it has the ability to react to all events that are reported to the
[DataReader](../dataReader/dataReader.md#dds-layer-subscriber-datareader).
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that SubscriberListener inherits from
DataReaderListener will only be called if the triggering DataReader has no Listener attached,
or if the callback is disabled by the [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) on the DataReader.

Additionally, SubscriberListener adds the following callback:

* [`on_data_on_readers()`](../../../api_reference/dds_pim/subscriber/subscriberlistener.md#_CPPv4N8eprosima7fastdds3dds18SubscriberListener18on_data_on_readersEP10Subscriber):
  New data is available on any DataReader
  belonging to this Subscriber.
  There is no queuing of invocations to this callback, meaning that if several new data changes are received
  at once, only one callback invocation may be issued for all of them, instead of one per change.
  If the application is retrieving the received data on this callback, it must keep
  [reading data](../dataReader/readingData.md#dds-layer-subscriber-accessreceived) until no new changes are left.

#### IMPORTANT
For more information about callbacks and its hierarchy, please refer to
[Listener](../../core/entity/entity.md#dds-layer-core-entity-commonchars-listener).

```c++
class CustomSubscriberListener : public SubscriberListener
{

public:

    CustomSubscriberListener()
        : SubscriberListener()
    {
    }

    virtual ~CustomSubscriberListener()
    {
    }

    virtual void on_data_on_readers(
            Subscriber* sub)
    {
        static_cast<void>(sub);
        std::cout << "New data available" << std::endl;
    }

};
```
