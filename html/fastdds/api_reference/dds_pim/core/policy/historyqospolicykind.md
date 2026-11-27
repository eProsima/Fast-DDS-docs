<a id="api-pim-historyqospolicykind"></a>

<a id="historyqospolicykind"></a>

# HistoryQosPolicyKind

### enum eprosima::fastdds::dds::HistoryQosPolicyKind

Enum HistoryQosPolicyKind, different kinds of History Qos for [HistoryQosPolicy](historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy). 

*Values:*

### enumerator KEEP_LAST_HISTORY_QOS

On the publishing side, the Service will only attempt to keep the most recent “depth” samples of each instance of data (identified by its key) managed by the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). On the subscribing side, the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) will only attempt to keep the most recent “depth” samples received for each instance (identified by its key) until the application “takes” them via the DataReader’s take operation. 

### enumerator KEEP_ALL_HISTORY_QOS

On the publishing side, the Service will attempt to keep all samples (representing each value written) of each instance of data (identified by its key) managed by the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) until they can be delivered to all subscribers. On the subscribing side, the Service will attempt to keep all samples of each instance of data (identified by its key) managed by the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader). These samples are kept until the application “takes” them from the Service via the take operation.
