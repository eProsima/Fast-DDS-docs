<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher-datawriterlistener"></a>

<a id="datawriterlistener"></a>

# DataWriterListener

[`DataWriterListener`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) is an abstract class defining the callbacks that will be triggered
in response to state changes on the [DataWriter](../dataWriter/dataWriter.md#dds-layer-publisher-datawriter).
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

[`DataWriterListener`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) defines the following callbacks:

* [`on_publication_matched()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener22on_publication_matchedEP10DataWriterRK24PublicationMatchedStatus): The DataWriter has found a
  [DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) that matches the [Topic](../../topic/topic/topic.md#dds-layer-topic-topic) and has
  a common partition and a compatible QoS, or has ceased to be matched with a
  DataReader that was previously considered to be matched.
* [`on_offered_deadline_missed()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener26on_offered_deadline_missedEP10DataWriterRK27OfferedDeadlineMissedStatus): The DataWriter failed to provide
  data within the deadline period configured on its [DataWriterQos](../dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos).
  It will be called for each deadline period and data instance for which the
  DataWriter failed to provide data.
* [`on_offered_incompatible_qos()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener27on_offered_incompatible_qosEP10DataWriterRK28OfferedIncompatibleQosStatus): The DataWriter has found a
  DataReader that matches the Topic and has
  a common partition, but with a requested QoS that is incompatible with the one defined on the
  DataWriter.
* [`on_liveliness_lost()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener18on_liveliness_lostEP10DataWriterRK20LivelinessLostStatus): The DataWriter did not respect the
  liveliness configuration on its DataWriterQos, and therefore,
  DataReader entities will consider the DataWriter
  as no longer *active*.
* [`on_unacknowledged_sample_removed()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener32on_unacknowledged_sample_removedEP10DataWriterRK16InstanceHandle_t): The Datawriter has removed a sample that has not been
  acknowledged by every matched DataReader.

<a id="dds-layer-publisher-datawriterlistener-on-unack-sample-removed"></a>

<a id="on-unacknowledged-sample-removed-callback"></a>

## on_unacknowledged_sample_removed callback

[`on_unacknowledged_sample_removed()`](../../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener32on_unacknowledged_sample_removedEP10DataWriterRK16InstanceHandle_t) non-standard callback notifies the user when a sample has
been removed without being sent/received by the matched DataReaders.
This could happen in constrained networks or if the publication throughput is too demanding.
This callback can be used to detect these situations so the publishing application can apply some solution to ease this
issue like reducing the publication rate.

The criteria to consider that a sample has been removed without being acknowledged depends on the
[ReliabilityQosPolicy](../../core/policy/standardQosPolicies.md#reliabilityqospolicy):

* [`BEST_EFFORT_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) DataWriters will notify that a sample has been removed while unacknowledged if the
  sample has not been sent through the transport.
* [`RELIABLE_RELIABILITY_QOS`](../../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) DataWriters consider samples to have been removed unacknowledged if not every matched
  DataReader has confirmed its reception by sending the corresponding meta-traffic `ACK` message.
  Consequently, a sample that is notified as removed unacknowledged might be received by one or more DataReaders, but
  not by every matched one, or at least, the `ACK` message has not been received at the moment of sample removal.
  A race condition is inevitable in this case, because when the sample is removed, the `ACK` from some matched
  DataReader is missing, but that means that it might have been lost in the transmission or that the message is still
  coming through and it will be received after the sample removal.
  Thus, this criteria may include false positives, but from the user’s point of view, it is more meaningful to know when
  the sample has not been acknowledged by every matched DataReader even if some samples are erroneously notified.

A specific case must be considered for reliable DataWriters with [DisablePositiveACKsQosPolicy](../../core/policy/eprosimaExtensions.md#disablepositiveacksqospolicy) enabled.
This policy disables the sending of positive `ACK` messages, unless the sample has been lost in which case the matched
DataReader notifies the loss with a negative `NACK` message.
If no `NACK` has been received in the time defined in the QoS policy, the sample is considered to be received.
Again, this is prone to race conditions because the `NACK` message might be on its way or have been lost in the
network.
For this specific case, where `ACK` messages are not going to be received, the reliable DataWriter uses the same
criteria as the best effort DataWriter.

```c++
class CustomDataWriterListener : public DataWriterListener
{

public:

    CustomDataWriterListener()
        : DataWriterListener()
    {
    }

    virtual ~CustomDataWriterListener()
    {
    }

    void on_publication_matched(
            DataWriter* writer,
            const PublicationMatchedStatus& info) override
    {
        static_cast<void>(writer);
        if (info.current_count_change == 1)
        {
            std::cout << "Matched a remote Subscriber for one of our Topics" << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "Unmatched a remote Subscriber" << std::endl;
        }
    }

    void on_offered_deadline_missed(
            DataWriter* writer,
            const OfferedDeadlineMissedStatus& status) override
    {
        static_cast<void>(writer);
        static_cast<void>(status);
        std::cout << "Some data could not be delivered on time" << std::endl;
    }

    void on_offered_incompatible_qos(
            DataWriter* writer,
            const OfferedIncompatibleQosStatus& status) override
    {
        std::cout << "Found a remote Topic with incompatible QoS (QoS ID: " << status.last_policy_id
                  << ")" << std::endl;
    }

    void on_liveliness_lost(
            DataWriter* writer,
            const LivelinessLostStatus& status) override
    {
        static_cast<void>(writer);
        static_cast<void>(status);
        std::cout << "Liveliness lost. Matched Subscribers will consider us offline" << std::endl;
    }

    void on_unacknowledged_sample_removed(
            DataWriter* writer,
            const InstanceHandle_t& instance) override
    {
        static_cast<void>(writer);
        static_cast<void>(instance);
        std::cout << "Sample removed unacknowledged" << std::endl;
    }

};
```
