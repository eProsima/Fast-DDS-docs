<a id="python-api-pim-matchedstatus"></a>

<a id="matchedstatus"></a>

# MatchedStatus

### *class* fastdds.MatchedStatus

A structure storing a matching status

#### *property* current_count

The number of writers currently matched to the concerned reader

#### *property* current_count_change

The change in current_count since the last time the listener was called or the status was read

#### *property* thisown

The membership flag

#### *property* total_count

Total cumulative count the concerned reader discovered a match with a writer It found a writer for the same topic with a requested QoS that is compatible with that offered by the reader

#### *property* total_count_change

The change in total_count since the last time the listener was called or the status was read
