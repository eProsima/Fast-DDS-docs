<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="property-policies-congestion-control"></a>

<a id="congestion-control-settings-pro"></a>

# Congestion Control Settings **[Pro]**

When using [Congestion Control](../use_cases/large_data/large_data.md#congestion-control) **[Pro]**, the DomainParticipant may need specific parameters
to be set.
Properties related with this feature lie on the `fastdds.congestion` namespace.
At least one of these properties should be set in order for the congestion control to be enabled.

Out-of-range values log a warning and revert to the property default.

<a id="common-properties"></a>

## Common properties

The following properties apply regardless of the selected plugin:

* Property `fastdds.congestion.plugin` is used to set the congestion control plugin to be used.
  Valid values are `basic` and `reno`.
  If the property is not present, it will be set to `basic`.
  See [congestion control plugins](../use_cases/large_data/large_data.md#congestion-control-plugins) for more information about the available plugins.
* Property `fastdds.congestion.period` is used to set the interval in milliseconds for checking whether the bandwidth
  to certain destinations should be updated.
  If the property is not present, it will be set to `10000` (10 seconds).
  The minimum valid value for this property is `1500` (1.5 seconds)
* Property `fastdds.congestion.initial_bandwidth` is used to set the bandwidth that will be initially assigned to
  new destinations, in bytes per second.
  The user must provide a value greater than the bandwidth required to send one message per second.
  If the property is not present, it will be set to half the size of the socket send buffer or the size of a message,
  whichever is greater.
* Property `fastdds.congestion.maximum_bandwidth` is used to set a hard upper cap, in bytes per second, on the
  bandwidth limit assigned to any destination.
  If the property is not present or set to `0`, no upper cap is applied.

<a id="basic-plugin-properties"></a>

## `basic` plugin properties

The following properties only apply when `fastdds.congestion.plugin` is `basic`:

* Property `fastdds.congestion.increase_multiplier` is used to set the multiplier that will be applied to increase
  the bandwidth to a destination when no congestion is detected.
  Thus, it should be a value greater than 1.
  If the property is not present, it will be set to `1.2`.
* Property `fastdds.congestion.decrease_multiplier` is used to set the multiplier that will be applied to decrease
  the bandwidth to a destination when congestion is detected.
  Thus, it should be a value between 0 and 1.
  If the property is not present, it will be set to `0.75`.

<a id="reno-plugin-properties"></a>

## `reno` plugin properties

The following properties only apply when `fastdds.congestion.plugin` is `reno`:

* Property `fastdds.congestion.initial_ss_threshold` sets the Slow-Start to Congestion-Avoidance handover level, in
  bytes per second.
  If the property is not present or set to `0`, it defaults to `4` times `initial_bandwidth`.
* Property `fastdds.congestion.slow_start_factor` sets the multiplicative growth applied to the limit per period
  during Slow Start.
  It should be a value greater than 1.
  If the property is not present, it will be set to `2.0`.
* Property `fastdds.congestion.additive_increment` sets the additive step, in bytes, applied to the limit per
  period during Congestion Avoidance and Fast Recovery.
  If the property is not present or set to `0`, it defaults to the participant’s maximum datagram size.
* Property `fastdds.congestion.fast_recovery_factor` sets the multiplicative decrease applied to the limit on a
  congestion episode.
  It should be a value between 0 and 1 (lower means a deeper, more conservative cut).
  If the property is not present, it will be set to `0.5`.
* Property `fastdds.congestion.timeout_recovery_factor` sets the multiplicative decrease applied to the limit on a
  stall (blackout).
  It should be a value between 0 and 1 and, since a stall is more severe than an isolated episode, smaller than
  `fast_recovery_factor`.
  If the property is not present, it will be set to `0.25`.
* Property `fastdds.congestion.repair_ratio_threshold` sets the retransmission-fraction level above which a
  worsening period counts as congestion.
  It should be a value between 0 and 1.
  If the property is not present, it will be set to `0.10`.
* Property `fastdds.congestion.ack_progress_timeout_periods` sets the number of consecutive blackout periods (data
  sent but nothing acknowledged and nothing retransmitted) that must elapse before the timeout event fires.
  It should be a value greater than 0.
  If the property is not present, it will be set to `3`.
* Property `fastdds.congestion.cap_binding_ratio` sets the fraction of the limit that the offered load must reach
  for the cap to count as binding.
  It should be a value between 0 and 1.
  If the property is not present, it will be set to `0.8`.
* Property `fastdds.congestion.worsening_margin` sets the relative drop of the delivered rate below its recent
  trend that marks delivery as worsening.
  It should be a value between 0 and 1.
  If the property is not present, it will be set to `0.1`.

More information about the `reno` plugin can be found in [reno plugin](../use_cases/large_data/large_data.md#reno-congestion-control-plugin).

<a id="datawriter-level-properties"></a>

## DataWriter-level properties

The following **DataWriter**-level property applies whichever value `fastdds.congestion.plugin` takes:

* Property `fastdds.sfc.bandwidth_reservation` sets the percentage of the transmitted bytes that retransmissions
  are entitled to, so that recovery is not starved by new samples.
  This share does not change when congestion control lowers the bandwidth limit, and it is not a hard partition: if
  no retransmissions are pending, new samples can use the whole bandwidth, and vice versa.
  It is configured per DataWriter, and when several DataWriters share the same congestion control the highest
  requested value applies.
  If the property is not present, it will be set to `30`.

  #### NOTE
  [`PRIORITY_WITH_RESERVATION`](../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy25PRIORITY_WITH_RESERVATIONE) flow controllers use this same property with a different
  meaning: there it reserves a percentage of the flow controller’s byte limit.
  See [Flow Controller Settings](flow_control.md#property-policies-flow-control).

<a id="examples"></a>

## Examples

The following is an example of how to set the `basic` congestion control up using the properties of
DomainParticipantQoS.

### C++

```c++
DomainParticipantQos participant_qos;

participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.plugin",
    "basic");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.period",
    "10000");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.initial_bandwidth",
    "100");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.increase_multiplier",
    "1.2");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.decrease_multiplier",
    "0.75");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.maximum_bandwidth",
    "0");  // 0 means unlimited
```

### XML

```xml
<!--
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
-->
        <participant profile_name="congestion_control_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>fastdds.congestion.plugin</name>
                            <value>basic</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.period</name>
                            <value>10000</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.initial_bandwidth</name>
                            <value>1000</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.increase_multiplier</name>
                            <value>1.2</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.decrease_multiplier</name>
                            <value>0.75</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.maximum_bandwidth</name>
                            <value>0</value>  <!-- 0 means unlimited -->
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
```

The following is an example of how to set the `reno` congestion control up using the properties of
DomainParticipantQoS.

### C++

```c++
DomainParticipantQos participant_qos;

participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.plugin",
    "reno");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.period",
    "100");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.initial_bandwidth",
    "650000");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.maximum_bandwidth",
    "0");  // 0 means unlimited
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.initial_ss_threshold",
    "41600000");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.slow_start_factor",
    "2.0");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.additive_increment",
    "262144");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.fast_recovery_factor",
    "0.5");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.timeout_recovery_factor",
    "0.25");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.repair_ratio_threshold",
    "0.05");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.ack_progress_timeout_periods",
    "3");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.cap_binding_ratio",
    "0.8");
participant_qos.properties().properties().emplace_back(
    "fastdds.congestion.worsening_margin",
    "0.1");

// Reserve, per DataWriter, part of the per-period budget for retransmissions
DataWriterQos writer_qos;
writer_qos.properties().properties().emplace_back(
    "fastdds.sfc.bandwidth_reservation",
    "30");
```

### XML

```xml
<!--
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
-->
        <participant profile_name="reno_congestion_control_domainparticipant_xml_profile">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>fastdds.congestion.plugin</name>
                            <value>reno</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.period</name>
                            <value>100</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.initial_bandwidth</name>
                            <value>650000</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.maximum_bandwidth</name>
                            <value>0</value>  <!-- 0 means unlimited -->
                        </property>
                        <property>
                            <name>fastdds.congestion.initial_ss_threshold</name>
                            <value>41600000</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.slow_start_factor</name>
                            <value>2.0</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.additive_increment</name>
                            <value>262144</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.fast_recovery_factor</name>
                            <value>0.5</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.timeout_recovery_factor</name>
                            <value>0.25</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.repair_ratio_threshold</name>
                            <value>0.05</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.ack_progress_timeout_periods</name>
                            <value>3</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.cap_binding_ratio</name>
                            <value>0.8</value>
                        </property>
                        <property>
                            <name>fastdds.congestion.worsening_margin</name>
                            <value>0.1</value>
                        </property>
                    </properties>
                </propertiesPolicy>
            </rtps>
        </participant>
        <!-- Reserve, per DataWriter, part of the per-period budget for retransmissions -->
        <data_writer profile_name="reno_congestion_control_datawriter_xml_profile">
            <propertiesPolicy>
                <properties>
                    <property>
                        <name>fastdds.sfc.bandwidth_reservation</name>
                        <value>30</value>
                    </property>
                </properties>
            </propertiesPolicy>
        </data_writer>
```
