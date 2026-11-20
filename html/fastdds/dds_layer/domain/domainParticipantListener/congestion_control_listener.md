<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-domain-congestioncontrollistener"></a>

<a id="congestioncontrollistener-pro"></a>

# CongestionControlListener **[Pro]**

`CongestionControlListener` is an abstract class that lets the application observe the
meta-information produced by the participant’s [congestion control](../../../use_cases/large_data/large_data.md#congestion-control)
mechanism.
It is purely observational, that is, implementing it does not change congestion-control behavior.

Congestion control is participant-scoped: there is a single controller per
[DomainParticipant](../domainParticipant/domainParticipant.md#dds-layer-domainparticipant), covering all reliable, non-builtin DataWriters and
evaluated independently for each remote DataReader.
The listener is therefore attached at the participant level through
[`set_congestion_control_listener()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant31set_congestion_control_listenerEPN7fastdds4rtps25CongestionControlListenerE), and only
takes effect when congestion control is enabled through the participant’s
[properties](../../../property_policies/congestion_control.md#property-policies-congestion-control).
For more information on congestion control, see [Congestion Control](../../../use_cases/large_data/large_data.md#congestion-control) .

As with other listeners, all callbacks have an empty default implementation, so the user
only needs to override the events of interest.

#### WARNING
The callbacks are invoked from the congestion-control evaluation thread.
The listener object must outlive the participant it is registered on (it is not copied),
and must not be destroyed while the participant is enabled.

<a id="callbacks"></a>

## Callbacks

`CongestionControlListener` defines the following callbacks:

* `on_cc_limit_update`: Called whenever the per-reader bandwidth limit changes, either an
  increase or a decrease.
  It receives a `CongestionControlLimitUpdateStatus` carrying the remote `reader_guid`,
  the `previous_limit` and `new_limit` (bytes per period) and the `direction` of the
  change (`INCREASE` or `DECREASE`).
* `on_cc_status_check`: Called once per remote reader on every periodic evaluation cycle,
  regardless of whether the limit changed.
  It receives a `CongestionControlStatus` snapshot with the remote `reader_guid`, the
  `current_limit`, the per-period byte counters (`period_sent_bytes`,
  `period_acked_bytes`, `period_resent_bytes`), the delivered throughput `acked_bps`,
  and the nominal `period_duration_ms`.
* `on_cc_info`: Generic algorithm-specific channel.
  It receives a `CongestionControlInfoStatus` with the remote `reader_guid` and a
  `{code, value}` pair whose meaning is defined by the active algorithm (see
  `CongestionControlInfoCodes.hpp`).
  For instance, the `basic` algorithm reports the bytes acknowledged, the percentage of
  bandwidth used, and the bytes resent during the period.

<a id="example"></a>

## Example

The following snippet shows a specialization of `CongestionControlListener`:

```c++
class CustomCongestionControlListener : public eprosima::fastdds::rtps::CongestionControlListener
{
    void on_cc_limit_update(
            const eprosima::fastdds::rtps::CongestionControlLimitUpdateStatus& status) override
    {
        std::cout << "Limit update for reader " << status.reader_guid
                  << ": " << status.previous_limit << " -> " << status.new_limit
                  << " (" << (status.direction ==
                eprosima::fastdds::rtps::CongestionControlLimitDirection::INCREASE
                              ? "increase" : "decrease") << ")" << std::endl;
    }

    void on_cc_status_check(
            const eprosima::fastdds::rtps::CongestionControlStatus& status) override
    {
        std::cout << "Status for reader " << status.reader_guid
                  << ": current_limit=" << status.current_limit
                  << ", acked_bps=" << status.acked_bps << std::endl;
    }

    void on_cc_info(
            const eprosima::fastdds::rtps::CongestionControlInfoStatus& status) override
    {
        // The meaning of the {code, value} pair depends on the active algorithm.
        // For the "basic" algorithm, the codes are defined in CongestionControlBasicInfoCode.
        switch (static_cast<eprosima::fastdds::rtps::CongestionControlBasicInfoCode>(status.code))
        {
            case eprosima::fastdds::rtps::CongestionControlBasicInfoCode::ACKS_RECEIVED:
                std::cout << "Reader " << status.reader_guid << " acked " << status.value
                          << " bytes during the period" << std::endl;
                break;
            case eprosima::fastdds::rtps::CongestionControlBasicInfoCode::BANDWIDTH_USED_PERCENT:
                std::cout << "Reader " << status.reader_guid << " used " << status.value
                          << "% of the current limit" << std::endl;
                break;
            case eprosima::fastdds::rtps::CongestionControlBasicInfoCode::RESENT_BYTES:
                std::cout << "Reader " << status.reader_guid << " had " << status.value
                          << " bytes resent during the period" << std::endl;
                break;
            default:
                std::cout << "Unknown congestion control info code: " << status.code << std::endl;
                break;
        }
    }

};
```

It can then be registered on a DomainParticipant on which congestion control has been
enabled:

```c++
        // Enable congestion control through the participant's properties
        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.properties().properties().emplace_back("fastdds.congestion.plugin", "basic");

        DomainParticipant* participant =
                DomainParticipantFactory::get_instance()->create_participant(0, pqos);
        if (nullptr == participant)
        {
            // Error
            return;
        }

        // Attach the listener (must outlive the participant)
        CustomCongestionControlListener cc_listener;
        participant->set_congestion_control_listener(&cc_listener);

        // ...

        // Detach the listener before destroying it
        participant->set_congestion_control_listener(nullptr);
```
