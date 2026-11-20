<a id="api-pim-matchedstatus"></a>

<a id="matchedstatus"></a>

# MatchedStatus

### struct MatchedStatus

A structure storing a matching status. 

Subclassed by [eprosima::fastdds::dds::PublicationMatchedStatus](publicationmatchedstatus.md#structeprosima_1_1fastdds_1_1dds_1_1PublicationMatchedStatus), [eprosima::fastdds::dds::SubscriptionMatchedStatus](subscriptionmatchedstatus.md#structeprosima_1_1fastdds_1_1dds_1_1SubscriptionMatchedStatus)

### Public Functions

### MatchedStatus() = default

Constructor. 

### ~MatchedStatus() = default

Destructor. 

### Public Members

### std::int32_t total_count = 0

Total cumulative count the concerned reader discovered a match with a writer. 

It found a writer for the same topic with a requested QoS that is compatible with that offered by the reader 

### std::int32_t total_count_change = 0

The change in total_count since the last time the listener was called or the status was read. 

### std::int32_t current_count = 0

The number of writers currently matched to the concerned reader. 

### std::int32_t current_count_change = 0

The change in current_count since the last time the listener was called or the status was read.
