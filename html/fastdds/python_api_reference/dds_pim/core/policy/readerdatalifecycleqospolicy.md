<a id="python-api-pim-readerdatalifecycleqospolicy"></a>

<a id="readerdatalifecycleqospolicy"></a>

# ReaderDataLifecycleQosPolicy

### *class* fastdds.ReaderDataLifecycleQosPolicy

Specifies the behavior of the DataReader with regards to the lifecycle of the data-instances it manages. Warning: This Qos Policy will be implemented in future releases. Notes: Mutable Qos Policy

#### *property* autopurge_disposed_samples_delay

Indicates the duration the DataReader must retain information regarding instances that have the
instance_state NOT_ALIVE_DISPOSED.

By default, dds::c_TimeInfinite.

#### *property* autopurge_no_writer_samples_delay

Indicates the duration the DataReader must retain information regarding instances that have the
instance_state NOT_ALIVE_NO_WRITERS.

By default, dds::c_TimeInfinite.

#### clear()

#### *property* thisown

The membership flag
