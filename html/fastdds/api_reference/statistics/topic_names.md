<a id="api-statistics-topic-names"></a>

<a id="topic-names"></a>

# Topic names

**[Pro]** In *Fast DDS* **[Pro]**, `NETWORK_LATENCY_TOPIC`, `RTPS_SENT_TOPIC`, `RTPS_LOST_TOPIC`, `DISCOVERY_TOPIC`,
`SAMPLE_DATAS_TOPIC` and `PHYSICAL_DATA_TOPIC` are deprecated and no longer published
(see [Statistics topics not available in Fast DDS](../../statistics/dds_layer/topic_names.md#statistics-topic-names-removed) ).

### constexpr const char \*eprosima::fastdds::statistics::HISTORY_LATENCY_TOPIC = "_fastdds_statistics_history2history_latency"

Statistic topic that reports the write-to-notification latency between any two pairs of matched DataWriter-DataReader histories 

### constexpr const char \*eprosima::fastdds::statistics::NETWORK_LATENCY_TOPIC = "_fastdds_statistics_network_latency"

The following statistics topics are no longer published, as nothing consumed the data they reported. The names are kept so that applications naming them still build, but enabling any of them has no effect other than logging a warning. Deprecated: it reported the transport latency between two locators. 

### constexpr const char \*eprosima::fastdds::statistics::PUBLICATION_THROUGHPUT_TOPIC = "_fastdds_statistics_publication_throughput"

Statistic topic that reports the publication’s throughput (amount of data sent) for every DataWriter. 

### constexpr const char \*eprosima::fastdds::statistics::SUBSCRIPTION_THROUGHPUT_TOPIC = "_fastdds_statistics_subscription_throughput"

Statistics topic that reports the subscription’s throughput (amount of data received) for every DataReader. 

### constexpr const char \*eprosima::fastdds::statistics::RTPS_SENT_TOPIC = "_fastdds_statistics_rtps_sent"

Deprecated: it reported the RTPS packets and bytes sent to each locator. 

### constexpr const char \*eprosima::fastdds::statistics::RTPS_LOST_TOPIC = "_fastdds_statistics_rtps_lost"

Deprecated: it reported the RTPS packets and bytes lost in the transport. 

### constexpr const char \*eprosima::fastdds::statistics::RESENT_DATAS_TOPIC = "_fastdds_statistics_resent_datas"

Statistics topic that reports the number of DATA/DATAFRAG sub-messages resent. 

### constexpr const char \*eprosima::fastdds::statistics::HEARTBEAT_COUNT_TOPIC = "_fastdds_statistics_heartbeat_count"

Statistics topic that reports the number of HEARTBEATs that each non discovery DataWriter sends. 

### constexpr const char \*eprosima::fastdds::statistics::ACKNACK_COUNT_TOPIC = "_fastdds_statistics_acknack_count"

Statistics topic that reports the number of ACKNACKs that each non discovery DataReader sends. 

### constexpr const char \*eprosima::fastdds::statistics::NACKFRAG_COUNT_TOPIC = "_fastdds_statistics_nackfrag_count"

Statistics topic that reports the number of NACKFRAGs that each non discovery DataReader sends. 

### constexpr const char \*eprosima::fastdds::statistics::GAP_COUNT_TOPIC = "_fastdds_statistics_gap_count"

Statistics topic that reports the number of GAPs that each non discovery DataWriter sends. 

### constexpr const char \*eprosima::fastdds::statistics::DATA_COUNT_TOPIC = "_fastdds_statistics_data_count"

Statistics topic that reports the number of DATA/DATAFRAG sub-messages that each non discovery DataWriter sends. 

### constexpr const char \*eprosima::fastdds::statistics::PDP_PACKETS_TOPIC = "_fastdds_statistics_pdp_packets"

Statistics topic that reports the number of PDP discovery traffic RTPS packets transmitted by each DDS participant. 

### constexpr const char \*eprosima::fastdds::statistics::EDP_PACKETS_TOPIC = "_fastdds_statistics_edp_packets"

Statistics topic that reports the number of EDP discovery traffic RTPS packets transmitted by each DDS participant. 

### constexpr const char \*eprosima::fastdds::statistics::DISCOVERY_TOPIC = "_fastdds_statistics_discovered_entity"

Deprecated: it reported the time when each remote entity was discovered. 

### constexpr const char \*eprosima::fastdds::statistics::SAMPLE_DATAS_TOPIC = "_fastdds_statistics_sample_datas"

Deprecated: it reported the submessages needed to deliver a single sample.
