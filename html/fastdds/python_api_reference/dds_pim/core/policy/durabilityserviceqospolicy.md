<a id="python-api-pim-durabilityserviceqospolicy"></a>

<a id="durabilityserviceqospolicy"></a>

# DurabilityServiceQosPolicy

### *class* fastdds.DurabilityServiceQosPolicy

Specifies the configuration of the durability service. That is, the service that implements the DurabilityQosPolicy kind
of TRANSIENT and PERSISTENT.

Warning: This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.

Notes: Immutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* history_depth

Number of most recent values that should be maintained on the History. It only have effect if the history_kind
is KEEP_LAST_HISTORY_QOS.

By default, 1.

#### *property* history_kind

Controls the HistoryQosPolicy of the fictitious DataReader that stores the data within the durability service.

By default, KEEP_LAST_HISTORY_QOS.

#### *property* max_instances

Control the ResourceLimitsQos of the implied DataReader that stores the data within the durability service.
Represents the maximum number of instances DataWriter (or DataReader) can manage.

By default, LENGTH_UNLIMITED.

#### *property* max_samples

Control the ResourceLimitsQos of the implied DataReader that stores the data within the durability service.
Specifies the maximum number of data-samples the DataWriter (or DataReader) can manage across all the instances
associated with it. Represents the maximum samples the middleware can store for any one DataWriter (or DataReader).
It is inconsistent for this value to be less than max_samples_per_instance.

By default, LENGTH_UNLIMITED.

#### *property* max_samples_per_instance

Control the ResourceLimitsQos of the implied DataReader that stores the data within the durability service.
Represents the maximum number of samples of any one instance a DataWriter(or DataReader) can manage.
It is inconsistent for this value to be greater than max_samples.

By default, LENGTH_UNLIMITED.

#### *property* service_cleanup_delay

Control when the service is able to remove all information regarding a data-instance.

By default, dds::c_TimeZero.

#### *property* thisown

The membership flag
