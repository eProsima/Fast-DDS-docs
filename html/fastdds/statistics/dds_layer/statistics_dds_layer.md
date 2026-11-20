<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="statistics-dds-layer"></a>

<a id="statistics-module-dds-layer"></a>

# Statistics Module DDS Layer

This section explains the DDS API provided for the *Statistics Module*.
First, the Statistics Topic List is presented together with the corresponding collected data.
Next, the methods to enable/disable the corresponding DataWriters are explained.
Then, the recommended QoS for enabling the DataWriters and creating the user’s DataReaders that subscribe to the
Statistics topics are described.
Finally, a guide on how to overcome common problems when using the module are presented.

* [Statistics Topic names](topic_names.md)
  * [`HISTORY_LATENCY_TOPIC`](topic_names.md#history-latency-topic)
  * [`NETWORK_LATENCY_TOPIC`](topic_names.md#network-latency-topic)
  * [`PUBLICATION_THROUGHPUT_TOPIC`](topic_names.md#publication-throughput-topic)
  * [`SUBSCRIPTION_THROUGHPUT_TOPIC`](topic_names.md#subscription-throughput-topic)
  * [`RTPS_SENT_TOPIC`](topic_names.md#rtps-sent-topic)
  * [`RTPS_LOST_TOPIC`](topic_names.md#rtps-lost-topic)
  * [`HEARTBEAT_COUNT_TOPIC`](topic_names.md#heartbeat-count-topic)
  * [`ACKNACK_COUNT_TOPIC`](topic_names.md#acknack-count-topic)
  * [`NACKFRAG_COUNT_TOPIC`](topic_names.md#nackfrag-count-topic)
  * [`GAP_COUNT_TOPIC`](topic_names.md#gap-count-topic)
  * [`DATA_COUNT_TOPIC`](topic_names.md#data-count-topic)
  * [`RESENT_DATAS_TOPIC`](topic_names.md#resent-datas-topic)
  * [`SAMPLE_DATAS_TOPIC`](topic_names.md#sample-datas-topic)
  * [`PDP_PACKETS_TOPIC`](topic_names.md#pdp-packets-topic)
  * [`EDP_PACKETS_TOPIC`](topic_names.md#edp-packets-topic)
  * [`DISCOVERY_TOPIC`](topic_names.md#discovery-topic)
  * [`PHYSICAL_DATA_TOPIC`](topic_names.md#physical-data-topic)
  * [Statistics topics not available in Fast DDS <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](topic_names.md#statistics-topics-not-available-in-fast-dds-pro)
* [Statistics Domain Participant](domainParticipant.md)
  * [Enable statistics DataWriters](domainParticipant.md#enable-statistics-datawriters)
  * [Disable statistics DataWriters](domainParticipant.md#disable-statistics-datawriters)
  * [Example](domainParticipant.md#example)
  * [Obtain pointer to the extended `statistics::dds::DomainParticipant` class](domainParticipant.md#obtain-pointer-to-the-extended-statisticsdomainparticipant-api-class)
  * [Automatically enabling statistics DataWriters](domainParticipant.md#automatically-enabling-statistics-datawriters)
* [Statistics recommended QoS](qos.md)
  * [Statistics DataWriter recommended QoS](qos.md#statistics-datawriter-recommended-qos)
  * [Statistics DataReader recommended QoS](qos.md#statistics-datareader-recommended-qos)
* [Troubleshooting](troubleshooting.md)
  * [Monitoring application is not receiving any statistic data](troubleshooting.md#monitoring-application-is-not-receiving-any-statistic-data)
