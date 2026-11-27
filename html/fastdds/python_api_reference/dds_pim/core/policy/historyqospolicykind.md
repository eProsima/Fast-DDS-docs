<a id="python-api-pim-historyqospolicykind"></a>

<a id="historyqospolicykind"></a>

# HistoryQosPolicyKind

### fastdds.KEEP_LAST_HISTORY_QOS

On the publishing side, the Service will only attempt to keep the most recent “depth” samples of each instance
of data (identified by its key) managed by the DataWriter. On the subscribing side, the DataReader will only attempt
to keep the most recent “depth” samples received for each instance (identified by its key) until the application
“takes” them via the DataReader’s take operation.

### fastdds.KEEP_ALL_HISTORY_QOS

On the publishing side, the Service will attempt to keep all samples (representing each value written) of each
instance of data (identified by its key) managed by the DataWriter until they can be delivered to all subscribers.
On the subscribing side, the Service will attempt to keep all samples of each instance of data (identified by its
key) managed by the DataReader. These samples are kept until the application “takes” them from the Service via the
take operation.
