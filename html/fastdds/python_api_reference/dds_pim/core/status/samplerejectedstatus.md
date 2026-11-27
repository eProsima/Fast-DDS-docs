<a id="python-api-pim-samplerejectedstatus"></a>

<a id="samplerejectedstatus"></a>

# SampleRejectedStatus

### *class* fastdds.SampleRejectedStatus

A struct storing the sample rejected status

#### *property* last_instance_handle

Handle to the instance being updated by the last sample that was rejected.

#### *property* last_reason

Reason for rejecting the last sample rejected.
If no samples have been rejected, the reason is the special value NOT_REJECTED.

#### *property* thisown

The membership flag

#### *property* total_count

Total cumulative count of samples rejected by the DataReader.

#### *property* total_count_change

The incremental number of samples rejected since the last time the listener was called or the status was read.
