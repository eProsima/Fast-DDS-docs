<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-datareaderlistener"></a>

<a id="datareaderlistener"></a>

# DataReaderListener

[`DataReaderListener`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE) is an abstract class defining the callbacks
that will be triggered in response to state changes on the [DataReader](../dataReader/dataReader.md#dds-layer-subscriber-datareader).
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

DataReaderListener defines the following callbacks:

* [`on_data_available()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader):
  There is new data available for the application on the DataReader.
  There is no queuing of invocations to this callback, meaning that if several new data changes are received
  at once, only one callback invocation may be issued for all of them, instead of one per change.
  If the application is retrieving the received data on this callback, it must keep
  [reading data](../dataReader/readingData.md#dds-layer-subscriber-accessreceived) until no new changes are left.
* [`on_subscription_matched()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener23on_subscription_matchedEP10DataReaderRKN7fastdds3dds25SubscriptionMatchedStatusE):
  The DataReader has found a
  [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) that matches the [Topic](../../topic/topic/topic.md#dds-layer-topic-topic) and has
  a common partition and a compatible QoS, or has ceased to be matched with a
  DataWriter that was previously considered to be matched.
  It is also triggered when a matched DataWriter has changed its
  [DataWriterQos](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos).
* [`on_requested_deadline_missed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener28on_requested_deadline_missedEP10DataReaderRK29RequestedDeadlineMissedStatus):
  The DataReader did not receive
  data within the deadline period configured on its [DataReaderQos](../dataReader/dataReader.md#dds-layer-subscriber-datareaderqos).
  It will be called for each deadline period and data instance for which the
  DataReader missed data.
* [`on_requested_incompatible_qos()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener29on_requested_incompatible_qosEP10DataReaderRK30RequestedIncompatibleQosStatus):
  The DataReader has found a
  DataWriter that matches the Topic and has
  a common partition, but with a QoS that is incompatible with the one defined on the
  DataReader.
* [`on_liveliness_changed()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener21on_liveliness_changedEP10DataReaderRK23LivelinessChangedStatus):
  The liveliness status of a matched DataWriter has changed.
  Either a DataWriter that was *inactive* has become *active* or the other
  way around.
* [`on_sample_rejected()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener18on_sample_rejectedEP10DataReaderRK20SampleRejectedStatus):
  A received data sample was rejected.
  See [SampleRejectedStatus](../../core/status/status.md#dds-layer-core-status-samplerejectedstatus) for further information.
* [`on_sample_lost()`](../../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener14on_sample_lostEP10DataReaderRK16SampleLostStatus):
  A data sample was lost and will never be received.
  See [SampleLostStatus](../../core/status/status.md#dds-layer-core-status-sampleloststatus) for further information.

#### IMPORTANT
For more information about callbacks and its hierarchy, please refer to
[Listener](../../core/entity/entity.md#dds-layer-core-entity-commonchars-listener).

```c++
class CustomDataReaderListener : public DataReaderListener
{

public:

    CustomDataReaderListener()
        : DataReaderListener()
    {
    }

    virtual ~CustomDataReaderListener()
    {
    }

    void on_data_available(
            DataReader* reader) override
    {
        static_cast<void>(reader);
        std::cout << "Received new data message" << std::endl;
    }

    void on_subscription_matched(
            DataReader* reader,
            const SubscriptionMatchedStatus& info) override
    {
        static_cast<void>(reader);
        if (info.current_count_change == 1)
        {
            std::cout << "Matched a remote DataWriter" << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "Unmatched a remote DataWriter" << std::endl;
        }
    }

    void on_requested_deadline_missed(
            DataReader* reader,
            const RequestedDeadlineMissedStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "Some data was not received on time" << std::endl;
    }

    void on_liveliness_changed(
            DataReader* reader,
            const LivelinessChangedStatus& info) override
    {
        static_cast<void>(reader);
        if (info.alive_count_change == 1)
        {
            std::cout << "A matched DataWriter has become active" << std::endl;
        }
        else if (info.not_alive_count_change == 1)
        {
            std::cout << "A matched DataWriter has become inactive" << std::endl;
        }
    }

    void on_sample_rejected(
            DataReader* reader,
            const SampleRejectedStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "A received data sample was rejected" << std::endl;
    }

    void on_requested_incompatible_qos(
            DataReader* reader,
            const RequestedIncompatibleQosStatus& info) override
    {
        std::cout << "Found a remote Topic with incompatible QoS (QoS ID: " << info.last_policy_id
                  << ")" << std::endl;
    }

    void on_sample_lost(
            DataReader* reader,
            const SampleLostStatus& info) override
    {
        static_cast<void>(reader);
        static_cast<void>(info);
        std::cout << "A data sample was lost and will not be received" << std::endl;
    }

};
```
