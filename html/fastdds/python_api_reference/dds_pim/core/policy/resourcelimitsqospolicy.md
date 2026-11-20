<a id="python-api-pim-resourcelimitsqospolicy"></a>

<a id="resourcelimitsqospolicy"></a>

# ResourceLimitsQosPolicy

### *class* fastdds.ResourceLimitsQosPolicy

Specifies the resources that the Service can consume in order to meet the requested QoS

Notes: Immutable Qos Policy

#### *property* allocated_samples

Number of samples initially allocated.

By default, 100.

#### clear()

Clears the QosPolicy object

#### *property* extra_samples

Represents the extra number of samples available once the max_samples have been reached in the history.
This makes it possible, for example, to loan samples even with a full history. By default, 1.

#### *property* max_instances

Represents the maximum number of instances DataWriter (or DataReader) can manage.

Value less or equal to 0 means infinite resources. By default, LENGTH_UNLIMITED.

Warning: It is inconsistent if (max_instances \* max_samples_per_instance) > max_samples.

#### *property* max_samples

Specifies the maximum number of data-samples the DataWriter (or DataReader) can manage across all the
instances associated with it. Represents the maximum samples the middleware can store for any one DataWriter
(or DataReader).

Value less or equal to 0 means infinite resources. By default, LENGTH_UNLIMITED.

Warning: It is inconsistent if max_samples < (max_instances \* max_samples_per_instance).

#### *property* max_samples_per_instance

Represents the maximum number of samples of any one instance a DataWriter(or DataReader) can manage.

Value less or equal to 0 means infinite resources. By default, LENGTH_UNLIMITED.

Warning: It is inconsistent if (max_instances \* max_samples_per_instance) > max_samples.

#### *property* thisown

The membership flag
